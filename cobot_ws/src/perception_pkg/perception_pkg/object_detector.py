#!/usr/bin/env python3
"""
object_detector.py

Finds the red cube and blue cylinder in the robot's camera image, works out
where they are in 3D space, and publishes that position relative to the
robot's base.

The pipeline, step by step:
  1. Get the color image and depth image (synchronized, so they match in time).
  2. Threshold the color image in HSV space to find "red" and "blue" pixels.
  3. Find the biggest blob of each color and take its center pixel (u, v).
  4. Look up the depth (distance from camera) at that pixel.
  5. Turn (u, v, depth) into an actual 3D point using the camera's intrinsics
     (the pinhole camera formula).
  6. Use TF2 to move that point from "camera frame" into "robot base frame",
     since that's the frame the rest of the robot stack cares about.
  7. Publish the result, plus an annotated debug image so you can see what
     the node is detecting.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge

import tf2_ros
from tf2_geometry_msgs import do_transform_point


# HSV color ranges for the two objects we're looking for.
# Hue in OpenCV goes from 0-179 (not 0-360). Pure red sits right at the 0/179
# wrap-around point, so we need two ranges to catch all of it.
RED_HSV_RANGES = [
    ((0, 120, 60), (10, 255, 255)),
    ((170, 120, 60), (179, 255, 255)),
]
BLUE_HSV_RANGES = [
    ((100, 120, 60), (130, 255, 255)),
]

# Colors are drawn in BGR (OpenCV's order) on the debug image.
DEBUG_BOX_COLOR = {
    'red_cube': (0, 0, 255),
    'blue_cylinder': (255, 0, 0),
}

# Ignore tiny specks of matching color (noise) -- a real cube/cylinder blob
# will be much bigger than this at any distance we care about.
MIN_BLOB_AREA_PX = 50

BASE_FRAME = 'base_link'


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()

        # Camera intrinsics -- we don't know these until the first CameraInfo
        # message arrives, so everything else has to wait for that.
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        # Color and depth need to be time-matched: we always want the depth
        # reading from the same instant as the color frame we detected in.
        color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.1)
        self.time_sync.registerCallback(self.image_callback)

        # TF2 buffer + listener: this is what lets us ask "where is this
        # camera-frame point, expressed in base_link frame instead?"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_publishers = {
            'red_cube': self.create_publisher(PoseStamped, '/detected_objects/red_cube', 10),
            'blue_cylinder': self.create_publisher(
                PoseStamped, '/detected_objects/blue_cylinder', 10),
        }
        self.debug_image_pub = self.create_publisher(Image, '/perception/debug_image', 10)

        self.get_logger().info('object_detector started, waiting for camera data...')

    def camera_info_callback(self, msg):
        # The 3x3 intrinsics matrix K is stored flattened as a 9-element list:
        #   [fx,  0, cx,
        #     0, fy, cy,
        #     0,  0,  1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_frame = msg.header.frame_id

    def image_callback(self, color_msg, depth_msg):
        if self.fx is None:
            return  # still waiting for camera_info

        color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        # 'passthrough' keeps the depth image as-is: 32-bit float meters per pixel.
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        debug_image = color_image.copy()
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        self.detect_and_publish(
            'red_cube', hsv_image, RED_HSV_RANGES, depth_image, color_msg.header, debug_image)
        self.detect_and_publish(
            'blue_cylinder', hsv_image, BLUE_HSV_RANGES, depth_image, color_msg.header,
            debug_image)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = color_msg.header
        self.debug_image_pub.publish(debug_msg)

    def detect_and_publish(self, name, hsv_image, hsv_ranges, depth_image, header, debug_image):
        mask = self.build_color_mask(hsv_image, hsv_ranges)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < MIN_BLOB_AREA_PX:
            return

        x, y, w, h = cv2.boundingRect(largest_contour)
        u = x + w // 2
        v = y + h // 2

        depth = float(depth_image[v, u])
        if not np.isfinite(depth) or depth <= 0.0:
            self.get_logger().warn(f'{name}: no valid depth reading at pixel ({u}, {v})')
            return

        point_in_base_frame = self.pixel_to_base_frame(u, v, depth, header)
        if point_in_base_frame is None:
            return  # TF lookup failed -- already logged inside the helper

        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = BASE_FRAME
        pose.pose.position = point_in_base_frame.point
        pose.pose.orientation.w = 1.0  # no meaningful orientation, so identity
        self.pose_publishers[name].publish(pose)

        box_color = DEBUG_BOX_COLOR[name]
        cv2.rectangle(debug_image, (x, y), (x + w, y + h), box_color, 2)
        cv2.circle(debug_image, (u, v), 4, box_color, -1)
        cv2.putText(debug_image, name, (x, max(y - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

    @staticmethod
    def build_color_mask(hsv_image, hsv_ranges):
        """OR together one or more HSV thresholds into a single binary mask."""
        mask = None
        for lower, upper in hsv_ranges:
            range_mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            mask = range_mask if mask is None else cv2.bitwise_or(mask, range_mask)
        return mask

    def pixel_to_base_frame(self, u, v, depth, header):
        """Pinhole projection (u, v, depth) -> 3D point, then TF2 into base_link."""
        point_in_camera = PointStamped()
        point_in_camera.header.frame_id = self.camera_frame
        point_in_camera.header.stamp = header.stamp
        point_in_camera.point.x = (u - self.cx) * depth / self.fx
        point_in_camera.point.y = (v - self.cy) * depth / self.fy
        point_in_camera.point.z = depth

        try:
            # Time() with no arguments means "use the latest transform available"
            # rather than the exact image timestamp -- simpler and avoids
            # extrapolation errors from small timing differences between nodes.
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME, self.camera_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup {self.camera_frame} -> {BASE_FRAME} failed: {e}')
            return None

        return do_transform_point(point_in_camera, transform)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
