#!/usr/bin/env python3
"""
YuMi MoveIt Demo Launch File

Launches YuMi robot with MoveIt for dual-arm planning and control.
Compatible with realbotics dual-arm teleop controller.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load a yaml file from a package"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except Exception as e:
        return None


def generate_launch_description():
    """Generate launch description for YuMi MoveIt demo"""

    # Declare arguments
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz?"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time"
    )

    # Get launch configurations
    rviz_config = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get package directories
    yumi_description_dir = get_package_share_directory("yumi_description")
    yumi_moveit_config_dir = get_package_share_directory("yumi_moveit_config")

    # Build file paths
    urdf_file = os.path.join(yumi_description_dir, "urdf", "yumi.urdf")
    srdf_file = os.path.join(yumi_moveit_config_dir, "config", "yumi.srdf")
    controllers_file = os.path.join(yumi_moveit_config_dir, "config", "moveit_controllers.yaml")
    joint_limits_file = os.path.join(yumi_moveit_config_dir, "config", "joint_limits.yaml")

    # Get configuration files
    kinematics_file = os.path.join(yumi_moveit_config_dir, "config", "kinematics.yaml")

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("yumi")
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .trajectory_execution(file_path=controllers_file)
        .joint_limits(file_path=joint_limits_file)
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    # RViz node
    rviz_config_file = os.path.join(yumi_moveit_config_dir, "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(rviz_config),
    )

    # Move Group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": True,
            },
        ],
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Joint State Publisher (fake controller)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"source_list": ["/move_group/fake_controller_joint_states"]},
        ],
    )

    # Static TF publisher for world frame
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "yumi_base_link"],
    )

    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        static_tf_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        move_group_node,
        rviz_node,
    ])
