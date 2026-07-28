import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    world_file = os.path.join(
        get_package_share_directory('arm_bringup'),
        'worlds',
        'table_scene.world'
    )

    controllers_file = PathJoinSubstitution(
        [FindPackageShare('arm_bringup'), 'config', 'ur_controllers.yaml']
    )

    initial_positions_file = PathJoinSubstitution(
        [FindPackageShare('ur_description'), 'config', 'initial_positions.yaml']
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ur_description'), 'rviz', 'view_robot.rviz']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    robot_description_content = Command([
         FindExecutable(name='xacro'), ' ',
         PathJoinSubstitution([
            FindPackageShare('arm_bringup'), 'urdf', 'cobra_arm.xacro'
         ]),
         ' ', 'ur_type:=ur5',
         ' ', 'name:=ur',
         ' ', 'sim_gazebo:=true',
         ' ', 'simulation_controllers:=', controllers_file,
         ' ', 'initial_positions_file:=', initial_positions_file,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(robot_description_content, value_type=str),
        }]
    )

    gripper_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('robotiq_description'), 'urdf', 'robotiq_2f_85_gripper.urdf.xacro'
        ]),
    ])

    gripper_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(gripper_description_content, value_type=str),
            'rate': 30,
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ur5', '-z', '0.75'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, rviz_node],
        )
    )

    delay_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        gripper_joint_state_publisher,
        spawn_entity,
        delay_controllers_after_spawn,
        delay_arm_controller_after_joint_state_broadcaster,
    ])
