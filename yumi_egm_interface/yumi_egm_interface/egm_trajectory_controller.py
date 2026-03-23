#!/usr/bin/env python3
"""
EGM Trajectory Controller for MoveIt2
=======================================
Replaces rws_trajectory_controller.py — instead of sending only the final point
via RWS/MoveAbsJ, this controller follows the ENTIRE trajectory point-by-point
at 250 Hz through EGM.

Provides FollowJointTrajectory action servers for:
  - /left_arm_controller/follow_joint_trajectory
  - /right_arm_controller/follow_joint_trajectory
  - /both_arms_controller/follow_joint_trajectory

Also publishes /joint_states at 250 Hz from EGM feedback (replaces the old
RWS-based joint_state_publisher that ran at 10 Hz).

Architecture:
  MoveIt2 → FollowJointTrajectory action → this node → EGM UDP → robot
  Robot → EGM UDP → this node → /joint_states → robot_state_publisher → TF

Fixes (2026-03-10):
  - both_arms: send to ALL arms first, then receive from ALL arms.
    Old code did send+receive per arm sequentially → 100ms per cycle instead of 4ms
    → EGM comm_timeout expired → robot didn't move.
  - Added _trajectory_lock to prevent two concurrent trajectory executions
    (e.g. MoveIt sending a second goal while first is still running) which caused
    conflicting EGM commands → violent shaking and large final errors.
"""

import math
import re
import threading
import time
from typing import List

import requests
from requests.auth import HTTPDigestAuth

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

from yumi_egm_interface.egm_manager import (
    EGMManager, EGMConfig, EGMArmChannel,
    LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES,
    LEFT_GRIPPER_JOINT, RIGHT_GRIPPER_JOINT,
)


class EGMTrajectoryController(Node):
    def __init__(self):
        super().__init__("egm_trajectory_controller")

        # ── Parameters ──
        self.declare_parameter("robot_ip", "192.168.125.1")
        self.declare_parameter("left_udp_port", 6511)
        self.declare_parameter("right_udp_port", 6512)
        self.declare_parameter("joint_state_rate", 250.0)  # Hz
        self.declare_parameter("trajectory_dt", 0.004)     # 4ms EGM cycle
        self.declare_parameter("final_error_tolerance_deg", 2.0)

        robot_ip = self.get_parameter("robot_ip").value
        left_port = self.get_parameter("left_udp_port").value
        right_port = self.get_parameter("right_udp_port").value
        self.js_rate = self.get_parameter("joint_state_rate").value
        self.traj_dt = self.get_parameter("trajectory_dt").value
        self.final_error_tolerance_deg = self.get_parameter("final_error_tolerance_deg").value

        # Sessão RWS para leitura de posição dos grippers
        self._rws_session = requests.Session()
        self._rws_session.auth = HTTPDigestAuth("Default User", "robotics")
        self._rws_base_io = f"http://{robot_ip}/rw/iosystem/signals"

        # ── EGM Manager ──
        config = EGMConfig(
            robot_ip=robot_ip,
            left_udp_port=left_port,
            right_udp_port=right_port,
        )
        self.egm = EGMManager(config)

        # ── Joint State Publisher ──
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.all_joint_names = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES + \
                               [LEFT_GRIPPER_JOINT, RIGHT_GRIPPER_JOINT]

        # ── Action Servers ──
        cb_group = ReentrantCallbackGroup()

        self.left_action = ActionServer(
            self, FollowJointTrajectory,
            "/left_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_left,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        self.right_action = ActionServer(
            self, FollowJointTrajectory,
            "/right_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_right,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        self.both_action = ActionServer(
            self, FollowJointTrajectory,
            "/both_arms_controller/follow_joint_trajectory",
            execute_callback=self._execute_both,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        # ── State ──
        self._executing_left = False
        self._executing_right = False
        self._cancel_requested = False

        # Global mutex: only ONE trajectory may run at a time across all arms.
        # Prevents MoveIt from sending a second goal while the first is still
        # executing, which would cause two threads to fight over the EGM sockets.
        self._trajectory_lock = threading.Lock()

        # Gripper positions (read from RWS or set to default)
        self._gripper_left_pos = 0.0
        self._gripper_right_pos = 0.0

        # Per-arm locks to guard EGM socket access between hold loop and exec loop
        self._left_lock = threading.Lock()
        self._right_lock = threading.Lock()

        # ── Start EGM ──
        self.get_logger().info("Starting EGM on both arms...")
        left_ok, right_ok = self.egm.start(timeout=10.0, start_background_threads=False)

        if left_ok:
            self.get_logger().info(
                f"LEFT arm EGM OK (port {left_port}, "
                f"eax_a={self.egm.left.initial_joints_egm[-1]:.1f}°)")
        else:
            self.get_logger().error(f"LEFT arm EGM FAILED on port {left_port}")

        if right_ok:
            self.get_logger().info(
                f"RIGHT arm EGM OK (port {right_port}, "
                f"eax_a={self.egm.right.initial_joints_egm[-1]:.1f}°)")
        else:
            self.get_logger().error(f"RIGHT arm EGM FAILED on port {right_port}")

        if not (left_ok and right_ok):
            self.get_logger().warn("Not all arms connected — some controllers may not work")

        # ── Start joint state publish timer ──
        period = 1.0 / self.js_rate
        self.js_timer = self.create_timer(period, self._publish_joint_states)

        # Timer para ler posição dos grippers via RWS (5 Hz é suficiente)
        self.gripper_timer = self.create_timer(0.2, self._update_gripper_positions)

        # ── Start EGM hold loops (sends hold when no trajectory is active) ──
        self._hold_thread_left = threading.Thread(
            target=self._hold_loop, args=(self.egm.left, "left"), daemon=True
        )
        self._hold_thread_right = threading.Thread(
            target=self._hold_loop, args=(self.egm.right, "right"), daemon=True
        )
        self._hold_thread_left.start()
        self._hold_thread_right.start()

        self.get_logger().info("EGM Trajectory Controller ready!")

    # ═══════════════════════════════════════════════════════════════
    # Joint State Publishing
    # ═══════════════════════════════════════════════════════════════

    def _publish_joint_states(self):
        """Publish joint states from EGM feedback at high rate."""
        left_state = self.egm.left.get_state()
        right_state = self.egm.right.get_state()

        if not left_state.valid and not right_state.valid:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.all_joint_names

        left_pos = left_state.joints_urdf_rad if left_state.valid else [0.0] * 7
        right_pos = right_state.joints_urdf_rad if right_state.valid else [0.0] * 7

        msg.position = left_pos + right_pos + [self._gripper_left_pos, self._gripper_right_pos]
        msg.velocity = []
        msg.effort = []

        self.js_pub.publish(msg)

    def _read_gripper_pos(self, lado: str) -> float:
        """Lê posição do gripper via RWS (0.1mm) e converte para metros."""
        try:
            r = self._rws_session.get(
                f"{self._rws_base_io}/hand_ActualPosition_{lado}", timeout=1
            )
            if r.status_code != 200:
                return None
            m = re.search(r'class="lvalue">([^<]+)<', r.text)
            return float(m.group(1)) / 10000.0 if m else None
        except Exception:
            return None

    def _update_gripper_positions(self):
        """Timer callback — atualiza posição dos grippers a 5 Hz via RWS."""
        pos_l = self._read_gripper_pos("L")
        pos_r = self._read_gripper_pos("R")
        if pos_l is not None:
            self._gripper_left_pos = pos_l
        if pos_r is not None:
            self._gripper_right_pos = pos_r

    # ═══════════════════════════════════════════════════════════════
    # Hold Loop — keeps EGM alive when no trajectory is active
    # ═══════════════════════════════════════════════════════════════

    def _hold_loop(self, channel: EGMArmChannel, arm_name: str):
        """Continuously receive from EGM and send hold when not executing."""
        executing_attr = f"_executing_{arm_name}"
        lock = self._left_lock if arm_name == "left" else self._right_lock

        while rclpy.ok():
            if not channel.is_connected:
                time.sleep(0.01)
                continue

            if getattr(self, executing_attr, False):
                time.sleep(0.01)
                continue

            with lock:
                channel.receive_and_update()
                state = channel.get_state()
                if state.valid:
                    channel.send_command(state.joints_egm_deg)

    # ═══════════════════════════════════════════════════════════════
    # Action Server Callbacks
    # ═══════════════════════════════════════════════════════════════

    def _goal_callback(self, goal_request):
        joint_names = set(goal_request.trajectory.joint_names)

        left_requested = any(j in LEFT_JOINT_NAMES for j in joint_names)
        right_requested = any(j in RIGHT_JOINT_NAMES for j in joint_names)

        if left_requested and not self.egm.left.is_connected:
            self.get_logger().error("Rejecting goal: left arm EGM not connected")
            return GoalResponse.REJECT

        if right_requested and not self.egm.right.is_connected:
            self.get_logger().error("Rejecting goal: right arm EGM not connected")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def _execute_left(self, goal_handle):
        return self._execute_trajectory(goal_handle, ["left"])

    async def _execute_right(self, goal_handle):
        return self._execute_trajectory(goal_handle, ["right"])

    async def _execute_both(self, goal_handle):
        return self._execute_trajectory(goal_handle, ["left", "right"])

    # ═══════════════════════════════════════════════════════════════
    # Trajectory Execution — the core loop
    # ═══════════════════════════════════════════════════════════════

    def _execute_trajectory(self, goal_handle, arms: List[str]):
        """
        Execute a trajectory on the specified arm(s).

        MoveIt2 sends a trajectory as a sequence of JointTrajectoryPoints with
        timestamps. We interpolate between them at 4ms intervals and send each
        interpolated point via EGM.

        KEY FIX 1 — both_arms send order:
          Send to ALL arms first, then receive from ALL arms. The old sequential
          send+receive per arm caused ~100ms cycles instead of 4ms.

        KEY FIX 2 — trajectory_lock:
          Only one trajectory may execute at a time. If MoveIt sends a second
          goal (e.g. plan+execute while a trajectory is still running), the new
          goal is immediately aborted instead of fighting over the EGM sockets.
        """
        result = FollowJointTrajectory.Result()

        # ── Acquire global trajectory lock (non-blocking) ──
        if not self._trajectory_lock.acquire(blocking=False):
            self.get_logger().error(
                f"Trajectory already executing — rejecting new goal for arms={arms}")
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Another trajectory is already executing"
            return result

        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        points = trajectory.points

        self.get_logger().info(
            f"Executing trajectory: {len(points)} points, "
            f"joints={joint_names}, arms={arms}")

        if len(points) == 0:
            self._trajectory_lock.release()
            goal_handle.succeed()
            return result

        self._cancel_requested = False

        for arm in arms:
            setattr(self, f"_executing_{arm}", True)

        try:
            # ── Build per-arm trajectory data ──
            arm_trajs = {}
            for arm in arms:
                arm_joint_names = LEFT_JOINT_NAMES if arm == "left" else RIGHT_JOINT_NAMES
                channel = self.egm.left if arm == "left" else self.egm.right
                lock = self._left_lock if arm == "left" else self._right_lock

                indices = []
                urdf_indices = []
                for i, jn in enumerate(joint_names):
                    if jn in arm_joint_names:
                        indices.append(i)
                        urdf_indices.append(arm_joint_names.index(jn))

                if not indices:
                    continue

                with lock:
                    state = channel.get_state()
                    if not state.valid:
                        self.get_logger().error(f"Arm {arm} state not valid, cannot execute")
                        continue
                    current_urdf = state.joints_urdf_rad.copy()

                waypoints = [(0.0, current_urdf.copy())]
                for pt in points:
                    t_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                    pos = current_urdf.copy()
                    for traj_idx, urdf_idx in zip(indices, urdf_indices):
                        if traj_idx < len(pt.positions):
                            pos[urdf_idx] = pt.positions[traj_idx]
                    waypoints.append((t_sec, pos))

                arm_trajs[arm] = {
                    "channel": channel,
                    "lock": lock,
                    "waypoints": waypoints,
                    "initial_urdf": current_urdf.copy(),
                }

            if not arm_trajs:
                self.get_logger().warn("No valid arm trajectories found")
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = "No valid arm trajectories found"
                return result

            # ── Execute trajectory at EGM rate ──
            total_time = max(v["waypoints"][-1][0] for v in arm_trajs.values())
            self.get_logger().info(f"Trajectory duration: {total_time:.3f}s")

            t_start = time.perf_counter()
            dt = self.traj_dt

            while True:
                t_elapsed = time.perf_counter() - t_start

                if t_elapsed > total_time + 0.5:
                    break

                if self._cancel_requested:
                    self.get_logger().info("Trajectory cancelled")
                    for _, data in arm_trajs.items():
                        with data["lock"]:
                            state = data["channel"].get_state()
                            if state.valid:
                                data["channel"].send_command(state.joints_egm_deg)
                    goal_handle.canceled()
                    return result

                t = min(t_elapsed, total_time)

                # STEP 1: Interpolate + send to ALL arms (no blocking)
                cmd_per_arm = {}
                for arm, data in arm_trajs.items():
                    pos_urdf = self._interpolate_waypoints(data["waypoints"], t)
                    vel_urdf = self._compute_velocity(data["waypoints"], t)
                    cmd_per_arm[arm] = (pos_urdf, vel_urdf)

                for arm, data in arm_trajs.items():
                    pos_urdf, vel_urdf = cmd_per_arm[arm]
                    with data["lock"]:
                        data["channel"].send_urdf_command(pos_urdf, vel_urdf)

                # STEP 2: Receive from ALL arms
                for arm, data in arm_trajs.items():
                    with data["lock"]:
                        data["channel"].receive_and_update()
                        state = data["channel"].get_state()
                        if not state.valid:
                            self.get_logger().error(
                                f"Arm {arm}: lost EGM feedback during execution")
                            goal_handle.abort()
                            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                            result.error_string = f"Lost EGM feedback on {arm} arm"
                            return result

                next_time = t_start + (math.floor(t_elapsed / dt) + 1) * dt
                sleep_time = next_time - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)

            # ── Check final position ──
            all_converged = True

            for arm, data in arm_trajs.items():
                target_urdf = data["waypoints"][-1][1]
                initial_urdf = data["initial_urdf"]

                with data["lock"]:
                    state = data["channel"].get_state()

                if not state.valid:
                    self.get_logger().error(f"Arm {arm}: invalid final EGM state")
                    all_converged = False
                    continue

                max_err = max(
                    abs(state.joints_urdf_rad[i] - target_urdf[i]) for i in range(7)
                )
                max_err_deg = math.degrees(max_err)
                self.get_logger().info(f"Arm {arm}: final error = {max_err_deg:.3f}°")

                commanded_motion = max(
                    abs(target_urdf[i] - initial_urdf[i]) for i in range(7)
                )
                actual_motion = max(
                    abs(state.joints_urdf_rad[i] - initial_urdf[i]) for i in range(7)
                )

                if commanded_motion > math.radians(5.0) and actual_motion < math.radians(1.0):
                    self.get_logger().error(
                        f"Arm {arm}: commanded motion but robot barely moved "
                        f"(cmd={math.degrees(commanded_motion):.2f}°, "
                        f"actual={math.degrees(actual_motion):.2f}°)"
                    )
                    all_converged = False

                if max_err_deg > self.final_error_tolerance_deg:
                    all_converged = False

            if all_converged:
                goal_handle.succeed()
                self.get_logger().info("Trajectory completed successfully")
                return result
            else:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "Final joint error exceeded tolerance"
                self.get_logger().warn("Trajectory completed with larger-than-expected error")
                return result

        except Exception as e:
            self.get_logger().error(f"Trajectory execution error: {e}")
            import traceback
            traceback.print_exc()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(e)
            return result

        finally:
            for arm in arms:
                setattr(self, f"_executing_{arm}", False)
            self._trajectory_lock.release()

    # ═══════════════════════════════════════════════════════════════
    # Trajectory Interpolation
    # ═══════════════════════════════════════════════════════════════

    def _interpolate_waypoints(self, waypoints: list, t: float) -> list:
        if t <= waypoints[0][0]:
            return waypoints[0][1].copy()
        if t >= waypoints[-1][0]:
            return waypoints[-1][1].copy()

        for i in range(len(waypoints) - 1):
            t0, p0 = waypoints[i]
            t1, p1 = waypoints[i + 1]
            if t0 <= t <= t1:
                if t1 - t0 < 1e-9:
                    return p1.copy()
                alpha = (t - t0) / (t1 - t0)
                return [p0[j] + alpha * (p1[j] - p0[j]) for j in range(len(p0))]

        return waypoints[-1][1].copy()

    def _compute_velocity(self, waypoints: list, t: float) -> list:
        if t <= waypoints[0][0] or t >= waypoints[-1][0]:
            return [0.0] * len(waypoints[0][1])

        for i in range(len(waypoints) - 1):
            t0, p0 = waypoints[i]
            t1, p1 = waypoints[i + 1]
            if t0 <= t <= t1:
                dt = t1 - t0
                if dt < 1e-9:
                    return [0.0] * len(p0)
                return [(p1[j] - p0[j]) / dt for j in range(len(p0))]

        return [0.0] * len(waypoints[0][1])

    # ═══════════════════════════════════════════════════════════════
    # Shutdown
    # ═══════════════════════════════════════════════════════════════

    def destroy_node(self):
        self.get_logger().info("Shutting down EGM...")
        self.egm.shutdown()
        self._rws_session.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EGMTrajectoryController()

    executor = MultiThreadedExecutor(num_threads=6)
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