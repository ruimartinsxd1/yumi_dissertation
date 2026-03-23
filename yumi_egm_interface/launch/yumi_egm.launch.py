#!/usr/bin/env python3
"""
Launch MoveIt2 com robô YuMi real via EGM (stack completo, 250 Hz).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # --- Argumentos ---
    rviz_arg        = DeclareLaunchArgument("rviz", default_value="false",
                                            description="Lançar RViz (true/false)")
    robot_ip_arg    = DeclareLaunchArgument("robot_ip", default_value="192.168.125.1",
                                            description="IP do controlador IRC5")
    left_port_arg   = DeclareLaunchArgument("left_udp_port", default_value="6511",
                                            description="Porta UDP EGM braço esquerdo")
    right_port_arg  = DeclareLaunchArgument("right_udp_port", default_value="6512",
                                            description="Porta UDP EGM braço direito")

    rviz_config = LaunchConfiguration("rviz")
    robot_ip    = LaunchConfiguration("robot_ip")
    left_port   = LaunchConfiguration("left_udp_port")
    right_port  = LaunchConfiguration("right_udp_port")

    # --- Paths ---
    yumi_desc_dir   = get_package_share_directory("yumi_description")
    yumi_moveit_dir = get_package_share_directory("yumi_moveit_config")

    urdf_file   = os.path.join(yumi_desc_dir,   "urdf",   "yumi.urdf")
    srdf_file   = os.path.join(yumi_moveit_dir, "config", "yumi.srdf")
    kin_file    = os.path.join(yumi_moveit_dir, "config", "kinematics.yaml")
    ctrl_file   = os.path.join(yumi_moveit_dir, "config", "moveit_controllers_egm.yaml")
    limits_file = os.path.join(yumi_moveit_dir, "config", "joint_limits.yaml")
    rviz_file   = os.path.join(yumi_moveit_dir, "config", "moveit.rviz")

    table_script = os.path.join(
        get_package_share_directory("yumi_rws_interface"), "scripts", "add_table_collision.py"
    )

    # --- MoveIt config ---
    moveit_config = (
        MoveItConfigsBuilder("yumi")
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kin_file)
        .trajectory_execution(file_path=ctrl_file)
        .joint_limits(file_path=limits_file)
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    # --- Nós ---

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "world", "yumi_base_link"],
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
    )

    rws_commander = Node(
        package="yumi_rws_interface",
        executable="rws_commander",
        name="yumi_rws_commander",
        parameters=[{
            "robot_ip": robot_ip,
            "state_publish_rate": 1.0,
        }],
        output="screen",
        emulate_tty=True,
    )

    egm_controller = Node(
        package="yumi_egm_interface",
        executable="egm_trajectory_controller",
        name="egm_trajectory_controller",
        parameters=[{
            "robot_ip": robot_ip,
            "left_udp_port": left_port,
            "right_udp_port": right_port,
            "joint_state_rate": 100.0,
            "trajectory_dt": 0.004,
        }],
        output="screen",
    )

    gripper_server = Node(
        package="yumi_rws_interface",
        executable="gripper_action_server",
        name="gripper_action_server",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "trajectory_execution.allowed_execution_duration_scaling": 3.0,
                "trajectory_execution.execution_duration_monitoring": False,
                "start_state_max_bounds_error": 0.05,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(rviz_config),
    )

    # Corre 8s após o launch para dar tempo ao MoveIt2 de arrancar
    table_collision = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=["python3", table_script],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        rviz_arg,
        robot_ip_arg,
        left_port_arg,
        right_port_arg,
        static_tf,
        robot_state_pub,
        rws_commander,
        egm_controller,
        gripper_server,
        move_group,
        rviz,
        table_collision,
    ])
