# 🦾 IntelliGrasp
### Language-Driven Visual Pick-and-Place for Collaborative Robots

> *Say what to pick. The robot sees it, plans the path, and grabs it — fully simulated in ROS 2.*

---

## 🧠 What Makes This Different

Most robot arm projects move joints with pre-coded coordinates.  
**IntelliGrasp bridges three worlds that are usually separate:**

```
"Pick the red cube"
       ↓
Computer Vision  →  3D Localisation  →  Motion Planning  →  Grasp
```

A text command drives the entire pipeline — no hardcoded positions, no manual waypoints.  
The robot *sees* the object, *finds* it in 3D space, *plans* the trajectory, and *executes* the pick — all automatically.

---

## 🎯 Project Goal

Build a fully simulated ROS 2 pick-and-place system where:

- A **UR5 industrial arm** is simulated in Gazebo
- An **RGB-D camera** detects coloured objects on a table using OpenCV
- A **depth image** converts the 2D detection into a real 3D world coordinate
- **TF2** transforms that coordinate from camera frame → robot base frame
- **MoveIt2** plans and executes the collision-free trajectory
- A **gripper** closes on the object and lifts it
- A **text command interface** (`/pick_object` topic) ties it all together

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────┐
│                   Gazebo Simulation                 │
│   UR5 Arm  +  Table  +  Objects  +  RGB-D Camera    │
└────────────────────────┬────────────────────────────┘
                         │
          ┌──────────────▼──────────────┐
          │       Perception Layer      │
          │  OpenCV colour detection    │
          │  Depth image → 3D point     │
          │  TF2: camera → base_link    │
          └──────────────┬──────────────┘
                         │
          ┌──────────────▼──────────────┐
          │      Motion Planning        │
          │  MoveIt2  +  IK solver      │
          │  Collision-aware trajectory │
          └──────────────┬──────────────┘
                         │
          ┌──────────────▼──────────────┐
          │     Execution & Gripper     │
          │  ros2_control joint cmds    │
          │  Open / close gripper       │
          └──────────────┬──────────────┘
                         │
          ┌──────────────▼──────────────┐
          │      Command Interface      │
          │  /pick_object "red cube"    │
          └─────────────────────────────┘
```

---

## 🗂️ Repository Structure

```
cobot_ws/
├── src/
│   ├── arm_description/       # URDF, Xacro, meshes, TF setup
│   ├── arm_gazebo/            # Gazebo world, launch files, config
│   ├── perception_pkg/        # OpenCV detection, depth→3D, TF2 node
│   ├── manipulation_pkg/      # MoveIt2 interface, pick-place logic
│   └── command_pkg/           # Text command subscriber node
├── .gitignore
└── README.md
```

---

## 🛠️ Tech Stack

| Layer | Tool |
|---|---|
| Simulation | Gazebo (Classic or Fortress) |
| Visualisation | RViz2 |
| Robot description | URDF / Xacro |
| Coordinate frames | TF2 |
| Computer vision | OpenCV 4 (HSV masking → bounding box) |
| 3D localisation | Depth image + camera intrinsics |
| Motion planning | MoveIt2 |
| Joint control | ros2_control |
| Language | Python 3 (rclpy) |
| ROS version | ROS 2 Humble |
| OS | Ubuntu 22.04 |

---

## 📅 Build Plan (5 Days)

| Day | Goal | Milestone |
|---|---|---|
| **1** | Workspace + GitHub + file structure | This repo exists and is clean |
| **2** | UR5 in Gazebo + RViz + camera attached | Arm visible, joints controllable |
| **3** | OpenCV detection + 3D pose published | Bounding boxes on camera feed, XYZ in terminal |
| **4** | MoveIt2 integration + trajectory execution | Arm moves to a target pose |
| **5** | Gripper + pick sequence + command interface | Full end-to-end demo: "pick red cube" → done |

---

## 🚀 Quick Start

> Prerequisites: Ubuntu 22.04, ROS 2 Humble, Gazebo, MoveIt2

```bash
# Clone the repo
git clone https://github.com/YOUR_USERNAME/intelligrasp.git
cd intelligrasp/cobot_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash

# Launch simulation (Day 2+)
ros2 launch arm_gazebo arm_world.launch.py

# Send a pick command (Day 5+)
ros2 topic pub /pick_object std_msgs/String "data: 'red cube'"
```

---

## 📦 Package Descriptions

### `arm_description`
Robot URDF and Xacro files for the UR5 arm. Includes the camera mount, gripper, and all TF frames from `base_link` through to `gripper_tip` and `camera_link`.

### `arm_gazebo`
Gazebo world file containing the table, coloured objects (red cube, blue cylinder, green sphere), and the spawning launch file. Configures ros2_control hardware interfaces for simulation.

### `perception_pkg`
The CV brain. Subscribes to `/camera/image_raw` and `/camera/depth/image_raw`. Runs OpenCV HSV colour masking to detect objects, uses the depth image to get the 3D centroid, and uses TF2 to transform it into the robot base frame. Publishes to `/detected_object_pose`.

### `manipulation_pkg`
The motion brain. Subscribes to `/detected_object_pose` and uses the MoveIt2 Python API to plan and execute a trajectory. Contains the full pick-and-place state machine: approach → descend → grasp → lift → retreat.

### `command_pkg`
The language interface. Subscribes to `/pick_object` (String topic). Parses the command to extract the target object colour and name, then triggers the perception → manipulation pipeline.

---

## 💡 Innovation

This project is not just "move a robot arm". It is a **minimal end-to-end cognitive pipeline**:

1. **Language understanding** — parse a natural command into a target descriptor
2. **Visual grounding** — find that object in a live camera feed
3. **Spatial reasoning** — lift a 2D pixel detection into 3D world space
4. **Embodied action** — plan a physically valid trajectory and execute it

Each component is a standalone ROS 2 node. Swap OpenCV for YOLO, swap the text interface for voice, or swap UR5 for any other arm — the architecture stays the same. That modularity is the real engineering contribution.

---

## 🔮 Future Work

- [ ] Replace HSV masking with YOLOv8 for robust multi-object detection
- [ ] Add voice command input (Whisper / Google STT)
- [ ] Dynamic obstacle avoidance with point cloud integration
- [ ] Multi-arm coordination
- [ ] Real hardware deployment (UR5e / Franka)

---

## 👤 Author - Ameed Nazhruudeen

Built as a 5-day ROS 2 learning project.  
Designed to be beginner-friendly while showcasing a real perception–planning–execution pipeline.

---

## 📄 License

MIT — free to use, fork, and build on.
