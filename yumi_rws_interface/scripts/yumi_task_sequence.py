#!/usr/bin/env python3
"""
yumi_task_sequence.py — Sequência de tarefa para inserção de fios RCD (modo cíclico).

v3 — Versão estável sequencial:
  - Abre ambos os grippers no início
  - Pausa entre movimentos para evitar error_code=-1
  - Retry automático em caso de falha
  - Modo cíclico até Ctrl+C

Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
FEUP/INESCTEC, 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
import yaml
import os
import time


# ============================================================
# CONFIGURAÇÃO
# ============================================================

POSES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.yaml")

JOINT_NAMES_LEFT = [
    "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l",
    "yumi_joint_3_l", "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l",
]
JOINT_NAMES_RIGHT = [
    "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r",
    "yumi_joint_3_r", "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r",
]

GRIPPER_OPEN_M        = 0.025
GRIPPER_CLOSE_M       = 0.0
MOVE_DURATION_S       = 5.0
PAUSE_BETWEEN_S       = 0.8
GRIPPER_PAUSE_S       = 1.0
PAUSE_BETWEEN_CYCLES  = False


# ============================================================
# Node principal
# ============================================================

class YuMiTaskNode(Node):

    def __init__(self):
        super().__init__("yumi_task_sequence")
        self._left_client  = ActionClient(self, FollowJointTrajectory,
                                          "/left_arm_controller/follow_joint_trajectory")
        self._right_client = ActionClient(self, FollowJointTrajectory,
                                          "/right_arm_controller/follow_joint_trajectory")
        self._gripper_l    = ActionClient(self, GripperCommand,
                                          "/left_gripper_controller/gripper_cmd")
        self._gripper_r    = ActionClient(self, GripperCommand,
                                          "/right_gripper_controller/gripper_cmd")

    def wait_servers(self, timeout=10.0):
        for name, client in [
            ("left_arm_controller",  self._left_client),
            ("right_arm_controller", self._right_client),
            ("left_gripper",         self._gripper_l),
            ("right_gripper",        self._gripper_r),
        ]:
            if not client.wait_for_server(timeout_sec=timeout):
                self.get_logger().error(f"Action server não disponível: {name}")
                return False
        return True

    def move_arm(self, side, joint_values, duration_s=MOVE_DURATION_S,
                 label="", retries=2):
        client      = self._left_client  if side == "left"  else self._right_client
        joint_names = JOINT_NAMES_LEFT   if side == "left"  else JOINT_NAMES_RIGHT

        self.get_logger().info(
            f"  → {label or side}: {[f'{v:.3f}' for v in joint_values]}")

        for attempt in range(retries):
            traj = JointTrajectory()
            traj.joint_names = joint_names
            pt = JointTrajectoryPoint()
            pt.positions  = [float(v) for v in joint_values]
            pt.velocities = [0.0] * len(joint_names)
            secs  = int(duration_s)
            nsecs = int((duration_s - secs) * 1e9)
            pt.time_from_start = Duration(sec=secs, nanosec=nsecs)
            traj.points = [pt]

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            for name in joint_names:
                tol = JointTolerance()
                tol.name     = name
                tol.position = 0.1
                tol.velocity = 0.5
                goal.path_tolerance.append(tol)

            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn(f"  ⚠ Goal rejeitado ({attempt+1}/{retries})")
                time.sleep(1.0)
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=duration_s + 10.0)
            result = result_future.result()
            if result and result.result.error_code == 0:
                self.get_logger().info(f"  ✓ {label} OK")
                return True
            else:
                code = result.result.error_code if result else "timeout"
                self.get_logger().warn(f"  ⚠ error_code={code} ({attempt+1}/{retries})")
                time.sleep(1.0)

        self.get_logger().error(f"  ✗ {label} falhou após {retries} tentativas")
        return False

    def gripper(self, side, position_m, label=""):
        client = self._gripper_l if side == "left" else self._gripper_r
        self.get_logger().info(
            f"  → Gripper {side}: {position_m*1000:.0f}mm ({label})")
        goal = GripperCommand.Goal()
        goal.command.position   = float(position_m)
        goal.command.max_effort = 0.0
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"  ✗ Gripper goal rejeitado ({side})")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        pos = result_future.result().result.position * 1000.0
        self.get_logger().info(f"  ✓ Gripper {side} OK — {pos:.1f}mm")
        return True


# ============================================================
# Sequência principal
# ============================================================

def run_task():
    rclpy.init()
    node = YuMiTaskNode()

    print("\n=== YuMi Task Sequence — RCD Wire Insertion ===\n")

    if not os.path.exists(POSES_FILE):
        print(f"✗ Ficheiro de poses não encontrado: {POSES_FILE}")
        print("  Corre primeiro: python3 record_poses.py")
        return

    with open(POSES_FILE) as f:
        poses = yaml.safe_load(f)

    print(f"✓ Poses carregadas: {list(poses.keys())}\n")

    print("A aguardar action servers...")
    if not node.wait_servers():
        print("✗ Action servers não disponíveis. Launch a correr?")
        return
    print("✓ Action servers prontos\n")

    def move(side, pose_name):
        p = poses[pose_name][side]
        ok = node.move_arm(side, p, label=pose_name)
        time.sleep(PAUSE_BETWEEN_S)
        return ok

    def grip(side, pos, label):
        ok = node.gripper(side, pos, label)
        time.sleep(GRIPPER_PAUSE_S)
        return ok

    # ---- Passo 0: Abrir grippers ----
    print("[0] Abrir ambos os grippers...")
    grip("left",  GRIPPER_OPEN_M, "abrir")
    grip("right", GRIPPER_OPEN_M, "abrir")

    ciclo = 0
    print("\nCtrl+C para parar.\n")

    try:
        while True:
            ciclo += 1
            print(f"\n{'='*50}")
            print(f"  CICLO {ciclo}")
            print(f"{'='*50}")

            print("\n[1] Braço direito → pick_approach")
            move("right", "right_pick_approach")

            print("\n[2] Braço direito → pick")
            move("right", "right_pick")

            print("\n[3] Fechar gripper direito (pegar fio)")
            grip("right", GRIPPER_CLOSE_M, "fechar")

            print("\n[4] Braço direito → handover")
            move("right", "handover_right")

            print("\n[5] Braço esquerdo → handover")
            move("left", "handover_left")

            print("\n[6] Handover — esquerdo fecha, direito abre")
            grip("left",  GRIPPER_CLOSE_M, "fechar")
            grip("right", GRIPPER_OPEN_M,  "abrir")

            print("\n[7] Braço esquerdo → insert_approach")
            move("left", "left_insert_approach")

            print("\n[8] Braço esquerdo → insert")
            move("left", "left_insert")

            print("\n[9] Abrir gripper esquerdo (largar fio)")
            grip("left", GRIPPER_OPEN_M, "abrir")

            print(f"\n✓ Ciclo {ciclo} concluído!")

            if PAUSE_BETWEEN_CYCLES:
                input("Prime Enter para o próximo ciclo (Ctrl+C para parar)...")

    except KeyboardInterrupt:
        print(f"\n\nParado após {ciclo} ciclo(s).")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_task()