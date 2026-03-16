#!/usr/bin/env python3
"""
joint_state_publisher.py — Nó ROS 2 que lê juntas reais via RWS e publica em /joint_states.

Mapeamento URDF ↔ RWS:
  URDF: yumi_joint_1, 2, 7, 3, 4, 5, 6  (junta 7 entre 2 e 3)
  RWS:  rax_1, rax_2, rax_3, rax_4, rax_5, rax_6, eax_a  (junta 7 no fim)
  Conversão: RWS graus → ROS radianos (math.radians)

Gripper:
  RWS: hand_ActualPosition (unidades 0.1mm) → URDF prismatic (metros)
  Conversão: valor / GRIPPER_POSITION_SCALE

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import math
import re

import requests
from requests.auth import HTTPDigestAuth

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Conversão de unidades do gripper: RWS devolve posição em incrementos de 0.1 mm
GRIPPER_POSITION_SCALE: float = 10000.0  # divide por isto para obter metros


class YuMiJointStatePublisher(Node):

    # Nomes das juntas no URDF (ordem do URDF)
    JOINT_NAMES_R = [
        "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r",
        "yumi_joint_3_r", "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r",
    ]
    JOINT_NAMES_L = [
        "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l",
        "yumi_joint_3_l", "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l",
    ]
    GRIPPER_NAMES = [
        "gripper_r_joint", "gripper_r_joint_m",
        "gripper_l_joint", "gripper_l_joint_m",
    ]

    # Mapeamento: para cada posição URDF, qual índice do array RWS usar
    # URDF: j1, j2, j7, j3, j4, j5, j6
    # RWS:  [0], [1], [6], [2], [3], [4], [5]
    RWS_TO_URDF_IDX = [0, 1, 6, 2, 3, 4, 5]

    def __init__(self):
        super().__init__("yumi_joint_state_publisher")

        # Parâmetros
        self.declare_parameter("robot_ip", "192.168.125.1")
        self.declare_parameter("username", "Default User")
        self.declare_parameter("password", "robotics")
        self.declare_parameter("rate_hz", 10.0)

        ip = self.get_parameter("robot_ip").value
        user = self.get_parameter("username").value
        pwd = self.get_parameter("password").value
        rate = self.get_parameter("rate_hz").value

        # Sessão HTTP (reutilizada — evita erro 503 "too many sessions")
        self.session = requests.Session()
        self.session.auth = HTTPDigestAuth(user, pwd)

        self.base_motion = f"http://{ip}/rw/motionsystem/mechunits"
        self.base_io = f"http://{ip}/rw/iosystem/signals"

        # Publisher
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        period = 1.0 / rate
        self.timer = self.create_timer(period, self.publish_joint_states)

        self.get_logger().info(
            f"YuMi Joint State Publisher iniciado — {rate} Hz, robot={ip}"
        )

    def read_rws_joints(self, mechunit: str):
        """Lê 7 juntas do RWS (graus) por chave. Retorna lista ou None se falhar."""
        try:
            r = self.session.get(
                f"{self.base_motion}/{mechunit}/jointtarget", timeout=2
            )
            if r.status_code != 200:
                return None
            pattern = r'class="(?P<key>rax_[1-6]|eax_a)">(?P<val>[^<]+)<'
            data = {}
            for k, v in re.findall(pattern, r.text):
                try:
                    data[k] = float(v)
                except Exception:
                    pass
            required = [f"rax_{i}" for i in range(1, 7)] + ["eax_a"]
            if any(k not in data for k in required):
                return None
            return [
                data["rax_1"], data["rax_2"], data["rax_3"],
                data["rax_4"], data["rax_5"], data["rax_6"], data["eax_a"],
            ]
        except Exception as e:
            self.get_logger().warn(f"Erro ao ler {mechunit}: {e}")
            return None

    def read_gripper_position(self, lado: str):
        """Lê posição do gripper (0.1mm) e converte para metros."""
        try:
            r = self.session.get(
                f"{self.base_io}/hand_ActualPosition_{lado}", timeout=2
            )
            if r.status_code != 200:
                return 0.0
            m = re.search(r'class="lvalue">([^<]+)<', r.text)
            if m:
                return float(m.group(1)) / GRIPPER_POSITION_SCALE  # 0.1mm → metros
            return 0.0
        except Exception:
            return 0.0

    def rws_to_urdf(self, rws_joints_deg):
        """Converte array RWS [j1..j6,j7] (graus) para ordem URDF (radianos)."""
        return [
            math.radians(rws_joints_deg[i]) for i in self.RWS_TO_URDF_IDX
        ]

    def publish_joint_states(self):
        """Callback do timer — lê juntas e publica."""
        joints_r_rws = self.read_rws_joints("ROB_R")
        joints_l_rws = self.read_rws_joints("ROB_L")

        if joints_r_rws is None or joints_l_rws is None:
            return  # Skip se não conseguir ler

        # Converter para ordem URDF e radianos
        joints_r = self.rws_to_urdf(joints_r_rws)
        joints_l = self.rws_to_urdf(joints_l_rws)

        # Ler grippers
        grip_r = self.read_gripper_position("R")
        grip_l = self.read_gripper_position("L")

        # Construir mensagem
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = self.JOINT_NAMES_R + self.JOINT_NAMES_L + self.GRIPPER_NAMES
        msg.position = (
            joints_r
            + joints_l
            + [grip_r, grip_r, grip_l, grip_l]  # _m espelha o principal
        )

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YuMiJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.session.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()