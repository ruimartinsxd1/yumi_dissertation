#!/usr/bin/env python3
"""
Launch YuMi IRB 14000 em simulação Gazebo Harmonic com MoveIt2 + ros2_control.

Estratégia:
  - Lê yumi.urdf (URDF flat existente, não modificado)
  - Remove plugin Gazebo Classic (libgazebo_ros_control.so) em memória
  - Injeta bloco <ros2_control> + plugin gz_ros2_control (Gazebo Harmonic)
  - Escreve URDF modificado em ficheiro temporário para o MoveItConfigsBuilder
  - Lança gz sim, spawn do robot, controllers, MoveIt2, RViz

Pré-requisitos:
  sudo apt install ros-jazzy-gz-ros2-control

Uso:
  ros2 launch yumi_rws_interface yumi_sim.launch.py
  ros2 launch yumi_rws_interface yumi_sim.launch.py rviz:=false
"""

import os
import tempfile
import atexit

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from moveit_configs_utils import MoveItConfigsBuilder


# =============================================================================
# Patch do URDF: Gazebo Classic → Gazebo Harmonic
# =============================================================================

def _patch_urdf_for_harmonic(urdf_path: str, gz_controllers_yaml: str) -> str:
    """
    Lê yumi.urdf e substitui o plugin Gazebo Classic pelo gz_ros2_control.
    Injeta bloco <ros2_control> com posições iniciais = posição home (SRDF).
    Não modifica o ficheiro original.
    """
    with open(urdf_path, 'r') as f:
        content = f.read()

    # --- Remove plugin Gazebo Classic (incompatível com Harmonic) ---
    old_plugin = (
        '  <gazebo>\n'
        '    <plugin filename="libgazebo_ros_control.so" name="gazebo_ros_controller">\n'
        '      <robotNamespace>/yumi</robotNamespace>\n'
        '    </plugin>\n'
        '  </gazebo>'
    )
    content = content.replace(old_plugin, '')

    # --- Bloco ros2_control (posições home do SRDF) ---
    # Braço esquerdo home: j1=0, j2=-2.2689, j7=2.3562, j3=0, j4=0.5236, j5=0, j6=0.6981
    # Braço direito home:  j1=0, j2=-2.2689, j7=-2.3562, j3=0, j4=0.5236, j5=0, j6=0.6981
    ros2_control_block = f"""
  <!-- ================================================================== -->
  <!-- ros2_control para Gazebo Harmonic (gz_ros2_control)                -->
  <!-- ================================================================== -->
  <ros2_control name="YuMiGazeboSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>

    <!-- Braço esquerdo -->
    <joint name="yumi_joint_1_l">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_2_l">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">-2.2689</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_7_l">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">2.3562</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_3_l">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_4_l">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.5236</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_5_l">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_6_l">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.6981</param></state_interface>
      <state_interface name="velocity"/>
    </joint>

    <!-- Braço direito -->
    <joint name="yumi_joint_1_r">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_2_r">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">-2.2689</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_7_r">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">-2.3562</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_3_r">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_4_r">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.5236</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_5_r">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="yumi_joint_6_r">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.6981</param></state_interface>
      <state_interface name="velocity"/>
    </joint>

    <!-- Garras (prismatic, 0–0.025 m) -->
    <joint name="gripper_l_joint">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="gripper_r_joint">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

  <!-- Plugin Gazebo Harmonic gz_ros2_control -->
  <gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>{gz_controllers_yaml}</parameters>
    </plugin>
  </gazebo>"""

    content = content.replace('</robot>', ros2_control_block + '\n</robot>')
    return content


# =============================================================================
# generate_launch_description
# =============================================================================

def generate_launch_description():

    # --- Paths de recursos para Gazebo Harmonic ---
    # Gazebo converte package:// → model:// e precisa saber onde encontrar yumi_description.
    # GZ_SIM_RESOURCE_PATH deve conter o directório-pai de yumi_description (share/).
    yumi_desc_prefix = get_package_prefix('yumi_description')
    gz_resource_share = os.path.join(yumi_desc_prefix, 'share')
    existing_gz_path  = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path  = (gz_resource_share + ':' + existing_gz_path
                         if existing_gz_path else gz_resource_share)

    # --- Argumentos ---
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Lançar RViz com config MoveIt2 (true/false)'
    )
    rviz_config = LaunchConfiguration('rviz')

    # --- Paths ---
    yumi_desc_dir    = get_package_share_directory('yumi_description')
    yumi_moveit_dir  = get_package_share_directory('yumi_moveit_config')
    yumi_rws_dir     = get_package_share_directory('yumi_rws_interface')
    ros_gz_sim_dir   = get_package_share_directory('ros_gz_sim')

    urdf_file           = os.path.join(yumi_desc_dir,   'urdf',   'yumi.urdf')
    srdf_file           = os.path.join(yumi_moveit_dir, 'config', 'yumi.srdf')
    kin_file            = os.path.join(yumi_moveit_dir, 'config', 'kinematics.yaml')
    limits_file         = os.path.join(yumi_moveit_dir, 'config', 'joint_limits.yaml')
    rviz_file           = os.path.join(yumi_moveit_dir, 'config', 'moveit.rviz')
    gz_controllers_yaml = os.path.join(yumi_rws_dir,    'config', 'gz_controllers.yaml')
    moveit_ctrl_sim     = os.path.join(yumi_rws_dir,    'config', 'moveit_controllers_sim.yaml')

    # --- Patch URDF (em memória, sem tocar no ficheiro original) ---
    patched_urdf = _patch_urdf_for_harmonic(urdf_file, gz_controllers_yaml)

    # Ficheiro temporário para o MoveItConfigsBuilder (lê de ficheiro, não de string)
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False, prefix='yumi_sim_'
    )
    tmp.write(patched_urdf)
    tmp.close()
    atexit.register(os.unlink, tmp.name)  # limpa ao sair

    # --- MoveIt2 config ---
    moveit_config = (
        MoveItConfigsBuilder('yumi')
        .robot_description(file_path=tmp.name)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kin_file)
        .trajectory_execution(file_path=moveit_ctrl_sim)
        .joint_limits(file_path=limits_file)
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    # =========================================================================
    # Nós
    # =========================================================================

    # --- Gazebo Harmonic (mundo vazio) ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # --- Ponte /clock Gazebo → ROS 2 (necessário para use_sim_time) ---
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # --- Robot State Publisher (publica TF e /robot_description) ---
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # --- Static TF: world → yumi_base_link ---
    # O SRDF define virtual_joint fixed world→yumi_base_link.
    # O Gazebo posiciona yumi_base_link na origem; esta TF liga ao frame "world".
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'yumi_base_link'],
        parameters=[{'use_sim_time': True}],
    )

    # --- Spawn robot no Gazebo (3 s = Gazebo já arrancou) ---
    gz_spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='gz_spawn_yumi',
                arguments=[
                    '-name', 'yumi',
                    '-topic', 'robot_description',
                    '-z', '0.0',
                ],
                output='screen',
            )
        ],
    )

    # --- Controller spawners (6–8 s = robot spawned + gz_ros2_control active) ---
    joint_state_broadcaster_spawner = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    left_arm_spawner = TimerAction(
        period=7.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    right_arm_spawner = TimerAction(
        period=7.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['right_arm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    left_gripper_spawner = TimerAction(
        period=7.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['left_gripper_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    right_gripper_spawner = TimerAction(
        period=7.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['right_gripper_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    # --- MoveIt2 move_group (9 s = controllers activos) ---
    move_group = TimerAction(
        period=9.0,
        actions=[Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {
                    'use_sim_time': True,
                    'trajectory_execution.allowed_execution_duration_scaling': 2.0,
                    'trajectory_execution.execution_duration_monitoring': False,
                },
            ],
        )],
    )

    # --- RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
        condition=IfCondition(rviz_config),
    )

    return LaunchDescription([
        rviz_arg,
        # Define GZ_SIM_RESOURCE_PATH antes de lançar o Gazebo
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        gz_sim,
        clock_bridge,
        robot_state_pub,
        static_tf,
        gz_spawn,
        joint_state_broadcaster_spawner,
        left_arm_spawner,
        right_arm_spawner,
        left_gripper_spawner,
        right_gripper_spawner,
        move_group,
        rviz,
    ])
