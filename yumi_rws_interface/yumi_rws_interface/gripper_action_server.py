#!/usr/bin/env python3
"""
gripper_action_server.py — Action server GripperCommand para SmartGrippers via RWS.

Fix (2026-03-10 v3):
  Usa sessão HTTP fresca por comando (não reutilizada) para evitar conflitos
  de estado/cookies com a sessão EGM. Sem mastership — command_input e
  target_position_input são PERS e escreváveis directamente via RWS.

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
FEUP/INESCTEC, 2026
"""

import re
import time
import threading
import requests
from requests.auth import HTTPDigestAuth

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import GripperCommand


GRIPPER_OPEN_MM: float = 25.0
GRIPPER_CLOSE_MM: float = 0.0
GRIPPER_OPEN_THRESHOLD_MM: float = 20.0
GRIPPER_CLOSE_THRESHOLD_MM: float = 0.5
GRIPPER_WAIT: float = 2.0
SIGNAL_PULSE_DELAY: float = 0.1


class GripperActionServer(Node):

    def __init__(self):
        super().__init__("gripper_action_server")

        self.declare_parameter("robot_ip", "192.168.125.1")
        self._ip = self.get_parameter("robot_ip").value
        self._auth = HTTPDigestAuth("Default User", "robotics")

        self.base_rapid = f"http://{self._ip}/rw/rapid/symbol/data/RAPID"
        self.base_io    = f"http://{self._ip}/rw/iosystem/signals"
        self._lock = threading.Lock()

        cb_group = ReentrantCallbackGroup()
        self._servers = {}
        for name, lado in [("left_gripper_controller", "L"), ("right_gripper_controller", "R")]:
            self._servers[name] = ActionServer(
                self, GripperCommand, f"/{name}/gripper_cmd",
                execute_callback=lambda gh, s=lado: self._execute(gh, s),
                goal_callback=self._goal_cb,
                cancel_callback=self._cancel_cb,
                callback_group=cb_group,
            )
            self.get_logger().info(f"Gripper action server: /{name}/gripper_cmd")

        self.get_logger().info(f"Gripper Action Server pronto — robot={self._ip}")
        threading.Thread(target=self._startup_init, daemon=True).start()

    def _new_session(self):
        """Cria uma sessão HTTP nova e independente para cada operação."""
        s = requests.Session()
        s.auth = self._auth
        return s

    def _startup_init(self):
        time.sleep(2.0)
        for lado in ["L", "R"]:
            self.get_logger().info(f"Gripper {lado}: g_Calibrate (cmd 2) no arranque...")
            ok = self._send_sg(lado, 2)
            if not ok:
                self.get_logger().error(f"Gripper {lado}: falha ao enviar cmd 2 via RWS")
            time.sleep(5.0)
            self.get_logger().info(f"Gripper {lado}: pronto")

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle, lado):
        target_mm = goal_handle.request.command.position * 1000.0

        if target_mm > GRIPPER_OPEN_THRESHOLD_MM:
            move_mm = GRIPPER_OPEN_MM
        elif target_mm < GRIPPER_CLOSE_THRESHOLD_MM:
            move_mm = GRIPPER_CLOSE_MM
        else:
            move_mm = target_mm

        self.get_logger().info(
            f"Gripper {lado}: target={target_mm:.1f}mm → moveTo={move_mm:.1f}mm")

        with self._lock:
            ok = self._send_sg(lado, 3, target_pos=move_mm)
            if not ok:
                self.get_logger().error(
                    f"Gripper {lado}: falha RWS ao enviar moveTo={move_mm}mm")

        time.sleep(GRIPPER_WAIT)

        pos = self._read_pos(lado)
        pos_m = pos / 1000.0 if pos is not None else 0.0

        result = GripperCommand.Result()
        result.position = pos_m
        result.reached_goal = True
        result.stalled = False

        feedback = GripperCommand.Feedback()
        feedback.position = pos_m
        feedback.reached_goal = True
        feedback.stalled = False
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        self.get_logger().info(f"Gripper {lado}: final={pos}mm")
        return result

    def _task(self, lado):
        return "T_ROB_L" if lado == "L" else "T_ROB_R"

    def _send_sg(self, lado, command, target_pos=None):
        """Envia comando ao TRobSG via RWS usando sessão fresca."""
        task = self._task(lado)
        sess = self._new_session()
        try:
            if target_pos is not None:
                r = sess.post(
                    f"{self.base_rapid}/{task}/TRobSG/target_position_input?action=set",
                    data={"value": str(target_pos)}, timeout=3
                )
                if r.status_code not in (200, 201, 204):
                    self.get_logger().warn(
                        f"RWS target_position_input → HTTP {r.status_code}")

            r = sess.post(
                f"{self.base_rapid}/{task}/TRobSG/command_input?action=set",
                data={"value": str(command)}, timeout=3
            )
            if r.status_code not in (200, 201, 204):
                self.get_logger().error(
                    f"RWS command_input={command} → HTTP {r.status_code}: {r.text[:200]}")
                return False

            # Pulsar sinal RUN_SG_ROUTINE
            sess.post(f"{self.base_io}/RUN_SG_ROUTINE?action=set",
                      data={"lvalue": "1"}, timeout=2)
            time.sleep(SIGNAL_PULSE_DELAY)
            sess.post(f"{self.base_io}/RUN_SG_ROUTINE?action=set",
                      data={"lvalue": "0"}, timeout=2)
            return True

        except Exception as e:
            self.get_logger().error(f"Gripper {lado} cmd={command}: {e}")
            return False
        finally:
            sess.close()

    def _read_pos(self, lado):
        sess = self._new_session()
        try:
            r = sess.get(f"{self.base_io}/hand_ActualPosition_{lado}", timeout=2)
            if r.status_code != 200:
                return None
            m = re.search(r'class="lvalue">([^<]+)<', r.text)
            return float(m.group(1)) / 10.0 if m else None
        except Exception as e:
            self.get_logger().warn(f"_read_pos {lado}: {e}")
            return None
        finally:
            sess.close()


def main(args=None):
    rclpy.init(args=args)
    node = GripperActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()