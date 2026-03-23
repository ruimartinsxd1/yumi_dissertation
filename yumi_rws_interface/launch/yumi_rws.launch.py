#!/usr/bin/env python3
"""
Launch MoveIt2 com robô YuMi real via RWS (stack completo).

Nós lançados:
  - joint_state_publisher    : feedback de joints a 10 Hz via RWS
  - rws_commander            : serviços motor on/off, RAPID start/stop, PP reset
  - rws_trajectory_controller: execução de trajetórias via RWS (envia waypoint final)
  - gripper_action_server    : GripperCommand actions para MoveIt2 (via RWS)
  - move_group               : MoveIt2 planeamento e execução
  - robot_state_publisher    : publica TF do robot
  - static_tf                : world → yumi_base_link
  - rviz2                    : visualização (opcional, rviz:=true/false)

Uso:
  ros2 launch yumi_rws_interface yumi_rws.launch.py
  ros2 launch yumi_rws_interface yumi_rws.launch.py robot_ip:=192.168.125.1 rviz:=false
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
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true",
                                     description="Lançar RViz (true/false)")
    robot_ip_arg = DeclareLaunchArgument("robot_ip", default_value="192.168.125.1",
                                         description="IP do controlador IRC5")

    rviz_config = LaunchConfiguration("rviz")
    robot_ip = LaunchConfiguration("robot_ip")

    # --- Paths ---
    yumi_desc_dir = get_package_share_directory("yumi_description")
    yumi_moveit_dir = get_package_share_directory("yumi_moveit_config")

    table_script = os.path.join(
        get_package_share_directory("yumi_rws_interface"), "scripts", "add_table_collision.py"
    )

    urdf_file   = os.path.join(yumi_desc_dir,    "urdf",   "yumi.urdf")
    srdf_file   = os.path.join(yumi_moveit_dir,  "config", "yumi.srdf")
    kin_file    = os.path.join(yumi_moveit_dir,  "config", "kinematics.yaml")
    ctrl_file   = os.path.join(yumi_moveit_dir,  "config", "moveit_controllers.yaml")
    limits_file = os.path.join(yumi_moveit_dir,  "config", "joint_limits.yaml")
    rviz_file   = os.path.join(yumi_moveit_dir,  "config", "moveit.rviz")

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

    joint_state_pub = Node(
        package="yumi_rws_interface",
        executable="joint_state_publisher",
        name="yumi_joint_state_publisher",
        parameters=[{
            "robot_ip": robot_ip,
            "rate_hz": 10.0,
        }],
        output="screen",
        emulate_tty=True,
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

    rws_controller = Node(
        package="yumi_rws_interface",
        executable="rws_trajectory_controller",
        name="rws_trajectory_controller",
        parameters=[{
            "robot_ip": robot_ip,
            "move_speed": "[100,50,100,50]",
            "waypoint_wait": 0.5,
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
                "trajectory_execution.allowed_execution_duration_scaling": 2.0,
                "trajectory_execution.execution_duration_monitoring": False,
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
        static_tf,
        robot_state_pub,
        joint_state_pub,
        rws_commander,
        rws_controller,
        gripper_server,
        move_group,
        rviz,
        table_collision,
    ])
