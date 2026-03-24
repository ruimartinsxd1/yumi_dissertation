#!/usr/bin/env python3
"""
EGM Communication Manager for ROS 2
=====================================
Manages EGM UDP connections for both YuMi arms.
Shared by the joint state publisher and the trajectory controller.

Handles:
  - UDP socket management (one per arm)
  - EGM start/stop via RWS signal pulses
  - Protobuf serialization/deserialization
  - Joint order mapping (URDF ↔ EGM)
  - Thread-safe access

Fix (2026-03-10):
  - _recv_loop now sends hold commands to keep EGM session alive (comm_timeout=5s)
  - _executing flag prevents hold loop from conflicting with trajectory execution
  - start_background_threads=False allows trajectory controller to manage its own loops
"""

import math
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import requests
from requests.auth import HTTPDigestAuth

from yumi_egm_interface import egm_pb2


@dataclass
class EGMConfig:
    """Configuration for the EGM manager."""
    robot_ip: str = "192.168.125.1"
    rws_user: str = "Default User"
    rws_pass: str = "robotics"
    left_udp_port: int = 6511      # Port for LEFT arm (eax_a ≈ +135°)
    right_udp_port: int = 6512     # Port for RIGHT arm (eax_a ≈ -135°)
    signal_start: str = "EGM_START_JOINT"
    signal_stop: str = "EGM_STOP"
    recv_timeout: float = 0.01     # UDP receive timeout [s]


@dataclass
class ArmState:
    """Current state of one arm from EGM feedback."""
    # Joint positions in EGM order: [rax_1, rax_2, rax_3, rax_4, rax_5, rax_6, eax_a]
    joints_egm_deg: list = field(default_factory=lambda: [0.0] * 7)
    # Joint positions in URDF order: [j1, j2, j7, j3, j4, j5, j6]
    joints_urdf_rad: list = field(default_factory=lambda: [0.0] * 7)
    timestamp: float = 0.0
    mci_state: int = 0       # 0=UNDEFINED, 1=ERROR, 2=STOPPED, 3=RUNNING
    motors_on: bool = False
    rapid_running: bool = False
    utilization: float = 0.0
    valid: bool = False


# ═══════════════════════════════════════════════════════════════════
# Joint Order Mapping
# ═══════════════════════════════════════════════════════════════════
#
# URDF joint order (kinematic chain):
#   [0] yumi_joint_1   (rax_1)
#   [1] yumi_joint_2   (rax_2)
#   [2] yumi_joint_7   (eax_a)  ← external axis, position 2 in URDF
#   [3] yumi_joint_3   (rax_3)
#   [4] yumi_joint_4   (rax_4)
#   [5] yumi_joint_5   (rax_5)
#   [6] yumi_joint_6   (rax_6)
#
# EGM feedback order:
#   joints:         [rax_1, rax_2, rax_3, rax_4, rax_5, rax_6]  (6 internal)
#   externalJoints: [eax_a]                                       (1 external)
#   Combined:       [rax_1, rax_2, rax_3, rax_4, rax_5, rax_6, eax_a]
#
# Mapping: EGM index → URDF index
#   EGM[0] rax_1 → URDF[0] j1
#   EGM[1] rax_2 → URDF[1] j2
#   EGM[2] rax_3 → URDF[3] j3
#   EGM[3] rax_4 → URDF[4] j4
#   EGM[4] rax_5 → URDF[5] j5
#   EGM[5] rax_6 → URDF[6] j6
#   EGM[6] eax_a → URDF[2] j7

# URDF joint names per arm (with prefix)
LEFT_JOINT_NAMES = [
    "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l",
    "yumi_joint_3_l", "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l",
]
RIGHT_JOINT_NAMES = [
    "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r",
    "yumi_joint_3_r", "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r",
]

# Gripper joint names (not controlled by EGM, published separately)
LEFT_GRIPPER_JOINT = "gripper_l_joint"
LEFT_GRIPPER_MIMIC_JOINT = "gripper_l_joint_m"
RIGHT_GRIPPER_JOINT = "gripper_r_joint"
RIGHT_GRIPPER_MIMIC_JOINT = "gripper_r_joint_m"


def egm_deg_to_urdf_rad(egm_joints_deg: list) -> list:
    """
    Convert EGM joint order (degrees) to URDF joint order (radians).

    EGM:  [rax_1, rax_2, rax_3, rax_4, rax_5, rax_6, eax_a]
    URDF: [j1,    j2,    j7,    j3,    j4,    j5,    j6   ]
    """
    if len(egm_joints_deg) != 7:
        return [0.0] * 7

    deg2rad = math.pi / 180.0
    return [
        egm_joints_deg[0] * deg2rad,  # URDF[0] j1 = EGM[0] rax_1
        egm_joints_deg[1] * deg2rad,  # URDF[1] j2 = EGM[1] rax_2
        egm_joints_deg[6] * deg2rad,  # URDF[2] j7 = EGM[6] eax_a
        egm_joints_deg[2] * deg2rad,  # URDF[3] j3 = EGM[2] rax_3
        egm_joints_deg[3] * deg2rad,  # URDF[4] j4 = EGM[3] rax_4
        egm_joints_deg[4] * deg2rad,  # URDF[5] j5 = EGM[4] rax_5
        egm_joints_deg[5] * deg2rad,  # URDF[6] j6 = EGM[5] rax_6
    ]


def urdf_rad_to_egm_deg(urdf_joints_rad: list) -> list:
    """
    Convert URDF joint order (radians) to EGM joint order (degrees).

    URDF: [j1,    j2,    j7,    j3,    j4,    j5,    j6   ]
    EGM:  [rax_1, rax_2, rax_3, rax_4, rax_5, rax_6, eax_a]
    """
    if len(urdf_joints_rad) != 7:
        return [0.0] * 7

    rad2deg = 180.0 / math.pi
    return [
        urdf_joints_rad[0] * rad2deg,  # EGM[0] rax_1 = URDF[0] j1
        urdf_joints_rad[1] * rad2deg,  # EGM[1] rax_2 = URDF[1] j2
        urdf_joints_rad[3] * rad2deg,  # EGM[2] rax_3 = URDF[3] j3
        urdf_joints_rad[4] * rad2deg,  # EGM[3] rax_4 = URDF[4] j4
        urdf_joints_rad[5] * rad2deg,  # EGM[4] rax_5 = URDF[5] j5
        urdf_joints_rad[6] * rad2deg,  # EGM[5] rax_6 = URDF[6] j6
        urdf_joints_rad[2] * rad2deg,  # EGM[6] eax_a = URDF[2] j7
    ]


# ═══════════════════════════════════════════════════════════════════
# EGM Arm Channel
# ═══════════════════════════════════════════════════════════════════

class EGMArmChannel:
    """Manages EGM UDP communication for a single arm."""

    def __init__(self, arm_name: str, udp_port: int, recv_timeout: float = 0.05):
        self.arm = arm_name
        self.port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", udp_port))
        self.sock.settimeout(recv_timeout)
        self.robot_addr: Optional[Tuple[str, int]] = None
        self.n_internal = 0
        self.n_external = 0
        self.seq = 1
        self._lock = threading.Lock()

        # Latest state
        self.state = ArmState()
        self._initial_joints_egm: Optional[list] = None

    def close(self):
        self.sock.close()

    @property
    def is_connected(self) -> bool:
        return self.robot_addr is not None and self.state.valid

    @property
    def initial_joints_egm(self) -> Optional[list]:
        return self._initial_joints_egm

    def receive_and_update(self) -> bool:
        """
        Receive one EgmRobot message, update internal state.
        Returns True if a valid message was received.
        """
        try:
            data, addr = self.sock.recvfrom(65535)
        except socket.timeout:
            return False

        msg = egm_pb2.EgmRobot()
        msg.ParseFromString(data)

        joints_egm = self._extract_joints(msg)
        if joints_egm is None:
            return False

        with self._lock:
            if self.robot_addr is None:
                self.robot_addr = addr
                self._initial_joints_egm = joints_egm.copy()

            if addr != self.robot_addr:
                return False

            self.state.joints_egm_deg = joints_egm
            self.state.joints_urdf_rad = egm_deg_to_urdf_rad(joints_egm)
            self.state.timestamp = time.time()
            self.state.valid = True

            if msg.HasField("mciState"):
                self.state.mci_state = msg.mciState.state
            if msg.HasField("motorState"):
                self.state.motors_on = (msg.motorState.state == 1)
            if msg.HasField("rapidExecState"):
                self.state.rapid_running = (msg.rapidExecState.state == 2)
            if msg.HasField("utilizationRate"):
                self.state.utilization = msg.utilizationRate

        return True

    def send_command(self, joints_egm_deg: list, joints_vel_deg: Optional[list] = None):
        """
        Send an EgmSensor command with joint positions and optional velocities.
        Joints must be in EGM order (degrees).
        """
        if self.robot_addr is None:
            return

        n_total = len(joints_egm_deg)
        if joints_vel_deg is None:
            joints_vel_deg = [0.0] * n_total

        msg = egm_pb2.EgmSensor()
        msg.header.seqno = self.seq
        msg.header.tm = int(time.time() * 1000) & 0xFFFFFFFF
        msg.header.mtype = egm_pb2.EgmHeader.MessageType.MSGTYPE_CORRECTION

        if self.n_internal > 0:
            msg.planned.joints.joints.extend(joints_egm_deg[:self.n_internal])
        if self.n_external > 0:
            msg.planned.externalJoints.joints.extend(
                joints_egm_deg[self.n_internal:self.n_internal + self.n_external])

        if self.n_internal > 0:
            msg.speedRef.joints.joints.extend(joints_vel_deg[:self.n_internal])
        if self.n_external > 0:
            msg.speedRef.externalJoints.joints.extend(
                joints_vel_deg[self.n_internal:self.n_internal + self.n_external])

        self.seq += 1
        self.sock.sendto(msg.SerializeToString(), self.robot_addr)

    def send_hold(self):
        """Send hold at current position (keeps EGM session alive)."""
        with self._lock:
            hold = self.state.joints_egm_deg.copy()
        if any(v != 0.0 for v in hold):  # only send if state is valid
            self.send_command(hold)

    def send_urdf_command(self, urdf_joints_rad: list, urdf_vel_rad: Optional[list] = None):
        """
        Send command with joints in URDF order (radians).
        Handles conversion to EGM order (degrees).
        """
        egm_deg = urdf_rad_to_egm_deg(urdf_joints_rad)
        egm_vel = None
        if urdf_vel_rad is not None:
            egm_vel = urdf_rad_to_egm_deg(urdf_vel_rad)
        self.send_command(egm_deg, egm_vel)

    def get_state(self) -> ArmState:
        """Get a thread-safe copy of the current state."""
        with self._lock:
            import copy
            return copy.deepcopy(self.state)

    def _extract_joints(self, msg) -> Optional[list]:
        """Extract joint values from EgmRobot, returns combined list or None."""
        def read(src):
            if src is None:
                return None
            n_int = len(src.joints.joints) if src.HasField("joints") else 0
            n_ext = len(src.externalJoints.joints) if src.HasField("externalJoints") else 0
            if n_int + n_ext == 0:
                return None
            self.n_internal = n_int
            self.n_external = n_ext
            joints = []
            if n_int:
                joints.extend(list(src.joints.joints))
            if n_ext:
                joints.extend(list(src.externalJoints.joints))
            return joints

        if msg.HasField("feedBack"):
            out = read(msg.feedBack)
            if out is not None:
                return out
        if msg.HasField("planned"):
            out = read(msg.planned)
            if out is not None:
                return out
        return None


# ═══════════════════════════════════════════════════════════════════
# EGM Manager
# ═══════════════════════════════════════════════════════════════════

class EGMManager:
    """
    High-level manager for dual-arm EGM communication.

    Handles:
    - RWS signal pulsing to start/stop EGM
    - Both arm channels
    - Background receive + hold thread (keeps EGM session alive)

    Usage by trajectory controller:
        # Pass start_background_threads=False to avoid socket race conditions
        # when the caller manages its own recv/hold loops.
        self.egm.start(timeout=10.0, start_background_threads=False)
    """

    def __init__(self, config: EGMConfig = None):
        self.config = config or EGMConfig()
        self._session = requests.Session()
        self._session.auth = HTTPDigestAuth(self.config.rws_user, self.config.rws_pass)

        self.left = EGMArmChannel("left", self.config.left_udp_port, self.config.recv_timeout)
        self.right = EGMArmChannel("right", self.config.right_udp_port, self.config.recv_timeout)

        self._running = False
        self._executing = False  # set True by trajectory controller during execution
        self._recv_thread_left: Optional[threading.Thread] = None
        self._recv_thread_right: Optional[threading.Thread] = None

    def start(self, timeout: float = 10.0, start_background_threads: bool = True) -> Tuple[bool, bool]:
        """
        Start EGM on both arms.
        Returns (left_ok, right_ok).

        Args:
            timeout: How long to wait for each arm to connect [s].
            start_background_threads: If True, launch internal recv+hold threads.
                Set to False when the caller (e.g. EGMTrajectoryController) manages
                its own recv/hold loops — avoids two threads racing on the same socket.
        """
        # Pulse start signal (shared signal triggers both tasks)
        self._pulse_signal(self.config.signal_start)

        # Wait for feedback from both arms
        left_ok = self._wait_for_arm(self.left, timeout)
        right_ok = self._wait_for_arm(self.right, timeout)

        if left_ok or right_ok:
            self._running = True
            if start_background_threads:
                # Start background receive + hold threads
                if left_ok:
                    self._recv_thread_left = threading.Thread(
                        target=self._recv_loop, args=(self.left,), daemon=True)
                    self._recv_thread_left.start()
                if right_ok:
                    self._recv_thread_right = threading.Thread(
                        target=self._recv_loop, args=(self.right,), daemon=True)
                    self._recv_thread_right.start()

        return left_ok, right_ok

    def stop(self):
        """Stop EGM on both arms."""
        self._running = False
        time.sleep(0.1)
        try:
            self._pulse_signal(self.config.signal_stop)
        except Exception:
            pass

    def shutdown(self):
        """Full shutdown: stop EGM, close sockets, close session."""
        self.stop()
        time.sleep(0.2)
        self.left.close()
        self.right.close()
        self._session.close()

    def _pulse_signal(self, signal_name: str, delay: float = 0.15):
        base_io = f"http://{self.config.robot_ip}/rw/iosystem/signals"
        url = f"{base_io}/{signal_name}?action=set"
        r1 = self._session.post(url, data={"lvalue": "1"}, timeout=2)
        r1.raise_for_status()
        time.sleep(delay)
        r0 = self._session.post(url, data={"lvalue": "0"}, timeout=2)
        r0.raise_for_status()

    def _wait_for_arm(self, channel: EGMArmChannel, timeout: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if channel.receive_and_update():
                if channel.state.mci_state == 1:  # MCI_ERROR
                    return False
                return True
        return False

    def _recv_loop(self, channel: EGMArmChannel):
        """
        Background receive loop.
        - Always receives incoming EGM feedback to keep state updated.
        - When NOT executing a trajectory, sends hold at current position
          to keep the EGM session alive (prevents comm_timeout expiry).
        - When _executing is True, backs off and lets the trajectory
          controller send commands exclusively.
        """
        while self._running:
            got_data = channel.receive_and_update()

            if self._executing:
                # Trajectory controller is active — don't interfere
                time.sleep(0.01)
                continue

            # Hold loop: send current position to keep EGM alive
            if got_data and channel.state.valid:
                channel.send_hold()
