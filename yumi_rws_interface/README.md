# yumi_rws_interface

ROS 2 interface for controlling the ABB YuMi IRB 14000 dual-arm robot via
Robot Web Services (RWS). Developed as part of the dissertation
**"Collaborative Robotics for RCD Pre-Assembly"** at FEUP/INESCTEC, 2026.

**Author:** Rui Martins (up202108756@edu.fe.up.pt)
**Supervisors:** Prof. Luís F. Rocha, Prof. [co-orientador]

---

## Package Description

This package provides the full ROS 2 ↔ RWS bridge for the YuMi:

- Publishes live joint states to `/joint_states` by polling the RWS HTTP API
- Exposes a `FollowJointTrajectory` action server so MoveIt2 can move the real robot
- Exposes a `GripperCommand` action server for SmartGripper control via MoveIt2
- Provides ROS 2 services for high-level robot control (RAPID start/stop, motors, PP reset)
- Includes standalone scripts for teach-and-replay and diagnostics

---

## Architecture

### Nodes

| Node | Entry Point | Description |
|------|------------|-------------|
| `yumi_joint_state_publisher` | `joint_state_publisher` | Reads joints from RWS at 10 Hz, publishes `/joint_states` |
| `yumi_rws_commander` | `rws_commander` | Services for RAPID/motor control, publishes `/yumi/controller_state` |
| `rws_trajectory_controller` | `rws_trajectory_controller` | FollowJointTrajectory action server — sends multi-waypoint MoveAbsJ to RWS |
| `gripper_action_server` | `gripper_action_server` | GripperCommand action server — controls SmartGrippers via TRobSG |
| `gripper_service` | `gripper_service` | Simple Trigger services for gripper open/close/init/calibrate |

### Topics

| Topic | Type | Publisher |
|-------|------|-----------|
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_publisher` |
| `/yumi/controller_state` | `std_msgs/String` | `rws_commander` |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/yumi/start_rapid` | `Trigger` | Start RAPID execution |
| `/yumi/stop_rapid` | `Trigger` | Stop RAPID execution |
| `/yumi/reset_program_pointer` | `Trigger` | Reset PP to main |
| `/yumi/set_motors` | `SetBool` | Motors ON (true) / OFF (false) — requires AUTO mode |
| `/yumi/get_state` | `Trigger` | Get full controller state |
| `/yumi/test_connection` | `Trigger` | Test RWS connectivity |
| `/{left\|right}_gripper/{init\|calibrate\|open\|close}` | `Trigger` | Direct gripper control |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/left_arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` | Left arm trajectory execution |
| `/right_arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` | Right arm trajectory execution |
| `/both_arms_controller/follow_joint_trajectory` | `FollowJointTrajectory` | Both arms trajectory execution |
| `/left_gripper_controller/gripper_cmd` | `GripperCommand` | Left gripper (MoveIt2) |
| `/right_gripper_controller/gripper_cmd` | `GripperCommand` | Right gripper (MoveIt2) |

### Communication Flow

```
MoveIt2
  │
  ├── /*/follow_joint_trajectory ──► rws_trajectory_controller ──► RWS HTTP ──► IRC5 RAPID
  │
  └── /*/gripper_cmd ──────────────► gripper_action_server ────────► RWS HTTP ──► TRobSG
                                                                        │
robot_state_publisher ◄── /joint_states ◄── joint_state_publisher ◄───┘
```

---

## Prerequisites

### Robot Setup
1. YuMi connected to the PC on the same network (default IP: `192.168.125.1`)
2. PC configured to IP `192.168.125.50` (or modify `config/yumi_rws.yaml`)
3. FlexPendant in **AUTO** mode
4. RAPID program running (`PP to Main` + `Play`)
5. Modules `TRobRAPID` and `TRobSG` loaded in both tasks (`T_ROB_L`, `T_ROB_R`)

### Software
- ROS 2 Jazzy on Ubuntu 24.04
- Python 3.12
- `python3-requests` (`pip install requests` or `apt install python3-requests`)
- MoveIt2 (for full motion planning stack)
- `yumi_description` and `yumi_moveit_config` packages (in `yumi_ros2/`)

---

## How to Launch

### 1. Basic RWS Bridge (joint states + services only)
```bash
ros2 launch yumi_rws_interface yumi_rws.launch.py robot_ip:=192.168.125.1
```
Starts: `joint_state_publisher` + `rws_commander`

### 2. Full MoveIt2 Stack with Real Robot
```bash
ros2 launch yumi_rws_interface yumi_moveit_real.launch.py robot_ip:=192.168.125.1
```
Starts: `robot_state_publisher` + `joint_state_publisher` + `rws_trajectory_controller`
       + `move_group` + `gripper_action_server` + RViz2

### 3. Standalone Tools
```bash
# Move right arm to a test position
ros2 run yumi_rws_interface move_yumi

# Move both arms to Home (calibration position)
ros2 run yumi_rws_interface move_yumi_home

# WASD keyboard control (right arm, joints 1-3)
ros2 run yumi_rws_interface move_yumi_wasd_no_rapid
```

---

## Scripts (Teach & Replay)

Located in `scripts/`. Run directly with Python (no ROS 2 required).

```bash
cd ~/yumi_ws/src/yumi_rws_interface/scripts/

# Interactive single-arm teach & replay
python3 teach_pick_place.py

# Replay with pre-recorded positions (no teaching phase)
python3 replay_pick_place.py

# Interactive dual-arm handover teach & replay
python3 teach_handover.py

# Diagnostics — discover available RWS endpoints
python3 test_rws_discovery.py [robot_ip]
```

Archived scripts (validated, kept for reference):
- `archive/replay_handover_safe.py` — Dual-arm handover with safety intermediate positions

---

## Useful Commands

```bash
# Build the package
cd ~/yumi_ws && colcon build --packages-select yumi_rws_interface --symlink-install

# Check running nodes
ros2 node list

# Check joint states
ros2 topic echo /joint_states

# Test connection to robot
ros2 service call /yumi/test_connection std_srvs/srv/Trigger

# Start RAPID
ros2 service call /yumi/start_rapid std_srvs/srv/Trigger

# Motors ON (requires AUTO mode)
ros2 service call /yumi/set_motors std_srvs/srv/SetBool "data: true"

# Open left gripper
ros2 service call /left_gripper/open std_srvs/srv/Trigger

# Close right gripper
ros2 service call /right_gripper/close std_srvs/srv/Trigger
```

---

## Technical Notes

### Joint Order Mapping (URDF ↔ RWS)

RWS returns joints as `[rax_1, rax_2, rax_3, rax_4, rax_5, rax_6, eax_a]`.
The URDF expects `[joint_1, joint_2, joint_7, joint_3, joint_4, joint_5, joint_6]`
(joint 7 = gripper rail, between joint 2 and 3 in the URDF chain).

Reorder index: `RWS_TO_URDF_IDX = [0, 1, 6, 2, 3, 4, 5]`

### Trajectory Strategy

`rws_trajectory_controller` receives full trajectories from MoveIt2 and executes
them via **`runMoveAbsJMulti`** — a RAPID routine that iterates over an array of
`jointtarget` waypoints.

**Why not send only the final point?**
MoveIt2 generates trajectories that avoid obstacles and respect joint limits along
the full path. Sending only the final point causes the IRC5 to go in a straight
joint-space move, ignoring the planned path and potentially causing collisions or
unexpected configurations.

**How it works:**
1. The trajectory from MoveIt2 can have hundreds of points — all are downsampled
   to at most `max_waypoints` (default: 16, hard limit: 18) using a joint-step
   threshold (`resample_max_joint_step_deg`, default: 6°): only points where any
   joint moves more than the threshold from the previous kept point are retained.
   The first and last points are always kept.
2. Each selected waypoint is written to the RAPID array `wp_array{i}` as a
   `jointtarget` value (`[[rax_1..rax_6],[eax_a,9E9,...]]`).
3. `wp_count` is set to the number of waypoints written.
4. `routine_name_input` is set to `"runMoveAbsJMulti"`.
5. Signal `RUN_RAPID_ROUTINE` is pulsed to trigger execution.

The RAPID side loops from `1` to `wp_count` and calls `MoveAbsJ wp_array{i}` for
each, executing the full trajectory on the controller.

**Additional safety features added:**
- `normalize_ext_axis`: adjusts `eax_a` (joint 7, gripper rail) in all waypoints
  to take the shortest path relative to the current position, avoiding 360° wraps.
- `max_jump_deg` (default: 220°): rejects the entire trajectory if any consecutive
  joint step exceeds the threshold, preventing unsafe moves from stale or corrupt
  waypoints.

### RAPID Trigger Mechanism

All motion commands use a "write variable → pulse signal" pattern:

**Multi-waypoint motion (`runMoveAbsJMulti`):**
1. Write `wp_array{1}` .. `wp_array{N}` — each a `jointtarget`
2. Write `wp_count` — number of waypoints
3. Write `wp_speed` — speed data (e.g. `[200,100,200,100]`)
4. Write `routine_name_input` = `"runMoveAbsJMulti"`
5. Pulse `RUN_RAPID_ROUTINE` high then low

**Single-point motion (`runMoveAbsJ`, legacy scripts):**
1. Write `move_jointtarget_input` — target `jointtarget`
2. Write `move_speed_input` and `routine_name_input` = `"runMoveAbsJ"`
3. Pulse `RUN_RAPID_ROUTINE` high then low

**Gripper commands:**
1. Write `command_input`
2. Pulse `RUN_SG_ROUTINE` high then low

The pulse delay (`SIGNAL_PULSE_DELAY = 0.1 s`) gives the IRC5 time to detect
the rising edge.

### Network Configuration

| Parameter | Value |
|-----------|-------|
| Robot IP | `192.168.125.1` |
| RWS Port | `80` (HTTP) |
| Authentication | HTTP Digest |
| Username | `Default User` |
| Password | `robotics` |
| RAPID Tasks | `T_ROB_L` (left), `T_ROB_R` (right) |
| Motion Module | `TRobRAPID` |
| Gripper Module | `TRobSG` |
