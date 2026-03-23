#!/usr/bin/env python3
"""
gripper_service.py — Nó ROS 2 com serviços de controlo dos SmartGrippers via RWS.

Expõe serviços Trigger para cada gripper (esquerdo e direito):
  /{left|right}_gripper/init       — Inicializa o gripper (cmd 1)
  /{left|right}_gripper/calibrate  — Calibra o gripper (cmd 2)
  /{left|right}_gripper/open       — Abre completamente (cmd 5, g_GripOut)
  /{left|right}_gripper/close      — Fecha completamente (cmd 4, g_GripIn)

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import re
import time
import requests
from requests.auth import HTTPDigestAuth

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


# Atraso do pulso do sinal digital (segundos) — tempo para o controlador detetar o pulso
SIGNAL_PULSE_DELAY: float = 0.1


class GripperService(Node):

    def __init__(self):
        super().__init__("gripper_service")

        self.declare_parameter("robot_ip", "192.168.125.1")
        ip = self.get_parameter("robot_ip").value

        self.session = requests.Session()
        self.session.auth = HTTPDigestAuth("Default User", "robotics")
        self.base_rapid = f"http://{ip}/rw/rapid/symbol/data/RAPID"
        self.base_io = f"http://{ip}/rw/iosystem/signals"

        # Services para cada gripper
        for lado in ["left", "right"]:
            L = lado[0].upper()  # "L" ou "R"
            self.create_service(Trigger, f"/{lado}_gripper/init",
                lambda req, res, s=L: self._cmd(req, res, s, 1, 3.0, "Init"))
            self.create_service(Trigger, f"/{lado}_gripper/calibrate",
                lambda req, res, s=L: self._cmd(req, res, s, 2, 3.0, "Calibrate"))
            self.create_service(Trigger, f"/{lado}_gripper/open",
                lambda req, res, s=L: self._cmd(req, res, s, 5, 2.0, "Open"))
            self.create_service(Trigger, f"/{lado}_gripper/close",
                lambda req, res, s=L: self._cmd(req, res, s, 4, 2.0, "Close"))

        self.get_logger().info(f"Gripper services prontos — robot={ip}")

    def _task(self, lado):
        return "T_ROB_L" if lado == "L" else "T_ROB_R"

    def _send_sg_command(self, lado, command, target_pos=None):
        task = self._task(lado)
        if target_pos is not None:
            self.session.post(
                f"{self.base_rapid}/{task}/TRobSG/target_position_input?action=set",
                data={"value": str(target_pos)}, timeout=3
            )
        self.session.post(
            f"{self.base_rapid}/{task}/TRobSG/command_input?action=set",
            data={"value": str(command)}, timeout=3
        )
        self.session.post(
            f"{self.base_io}/RUN_SG_ROUTINE?action=set",
            data={"lvalue": "1"}, timeout=2
        )
        time.sleep(SIGNAL_PULSE_DELAY)
        self.session.post(
            f"{self.base_io}/RUN_SG_ROUTINE?action=set",
            data={"lvalue": "0"}, timeout=2
        )

    def _read_gripper(self, lado):
        try:
            r = self.session.get(
                f"{self.base_io}/hand_ActualPosition_{lado}", timeout=2
            )
            m = re.search(r'class="lvalue">([^<]+)<', r.text)
            return float(m.group(1)) / 10.0 if m else None
        except:
            return None

    def _cmd(self, request, response, lado, command, wait, name):
        try:
            self._send_sg_command(lado, command)
            time.sleep(wait)
            pos = self._read_gripper(lado)
            response.success = True
            response.message = f"{name} {lado} OK — pos={pos}mm"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Erro: {e}"
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperService()
    rclpy.spin(node)
    node.session.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
