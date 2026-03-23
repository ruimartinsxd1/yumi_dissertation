# yumi_rws_interface

ROS 2 interface for controlling the ABB YuMi IRB 14000 dual-arm robot via
Robot Web Services (RWS) — HTTP polling at 10 Hz.
Developed as part of the dissertation **"Collaborative Robotics for RCD Pre-Assembly"**
at FEUP/INESCTEC, 2026.

**Author:** Rui Martins (up202108756@edu.fe.up.pt)
**Supervisors:** Prof. Luís F. Rocha and Carlos M Costa

> For real-time EGM control (250 Hz UDP), see the companion package
> [`yumi_egm_interface`](../yumi_egm_interface/).

---

## Package Description

This package provides the RWS (HTTP) half of the ROS 2 ↔ ABB YuMi bridge:

- Publishes live joint states to `/joint_states` at 10 Hz via RWS polling
- Exposes `FollowJointTrajectory` action servers for MoveIt2 (RAPID MoveAbsJ via HTTP)
- Exposes `GripperCommand` action servers for SmartGripper control via MoveIt2
- Provides ROS 2 services for high-level robot control (RAPID start/stop, motors, PP reset)
- Includes standalone scripts for pose teaching, task execution, and diagnostics
- Also used by `yumi_egm_interface` for gripper control and EGM start/stop signals

| Mode | Launch file | Joint state rate | Trajectory execution |
|------|-------------|-----------------|----------------------|
| **RWS** | `yumi_rws.launch.py` | 10 Hz (HTTP polling) | RAPID `MoveAbsJ` via HTTP variable writes |
| **EGM** | `yumi_egm_interface` → `yumi_egm.launch.py` | 100–250 Hz (UDP) | Real-time interpolation at 4 ms cycles |

---

## Architecture

### Nodes

| Node | Entry point | Description |
|------|------------|-------------|
| `yumi_joint_state_publisher` | `joint_state_publisher` | Reads joints from RWS at 10 Hz, publishes `/joint_states` (RWS mode only) |
| `yumi_rws_commander` | `rws_commander` | Services for RAPID/motor control, publishes `/yumi/controller_state` |
| `rws_trajectory_controller` | `rws_trajectory_controller` | FollowJointTrajectory action server — sends multi-waypoint MoveAbsJ to RWS (RWS mode) |
| `egm_trajectory_controller` | `egm_trajectory_controller` | FollowJointTrajectory action server + `/joint_states` publisher via EGM UDP (EGM mode) |
| `gripper_action_server` | `gripper_action_server` | GripperCommand action server — controls SmartGrippers via TRobSG |
| `gripper_service` | `gripper_service` | Simple Trigger services for gripper open/close/init/calibrate |

### Topics

| Topic | Type | Publisher |
|-------|------|-----------|
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_publisher` (RWS) or `egm_trajectory_controller` (EGM) |
| `/yumi/controller_state` | `std_msgs/String` | `rws_commander` @ 1 Hz |

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

**RWS mode:**
```
MoveIt2
  │
  ├── /*/follow_joint_trajectory ──► rws_trajectory_controller ──► RWS HTTP ──► IRC5 RAPID
  │
  └── /*/gripper_cmd ──────────────► gripper_action_server ────────► RWS HTTP ──► TRobSG
                                                                        │
robot_state_publisher ◄── /joint_states ◄── joint_state_publisher ◄───┘
```

**EGM mode:**
```
MoveIt2
  │
  ├── /*/follow_joint_trajectory ──► egm_trajectory_controller ──► UDP 6511/6512 ──► IRC5 EGM
  │                                         │
  │                                         ├── /joint_states (250 Hz, from EGM feedback)
  │                                         └── gripper positions (5 Hz, from RWS)
  │
  └── /*/gripper_cmd ──────────────► gripper_action_server ──────► RWS HTTP ──► TRobSG
```

---

## Prerequisites

### Robot Setup
1. YuMi connected to the PC on the same network (default IP: `192.168.125.1`)
2. PC configured to IP `192.168.125.50` (or pass `robot_ip:=<ip>` at launch)
3. FlexPendant in **AUTO** mode
4. RAPID running (`PP to Main` + `Play`)
5. Modules `TRobRAPID` and `TRobSG` loaded in both tasks (`T_ROB_L`, `T_ROB_R`)

### Software
- ROS 2 Jazzy on Ubuntu 24.04
- Python 3.12
- `python3-requests` (`apt install python3-requests`)
- MoveIt2 (`apt install ros-jazzy-moveit`)
- `yumi_description` and `yumi_moveit_config` packages (in `../yumi_ros2/`)

---

## How to Launch

### 1. RWS mode — full MoveIt2 stack via HTTP

```bash
ros2 launch yumi_rws_interface yumi_rws.launch.py robot_ip:=192.168.125.1
```

Starts: `robot_state_publisher` + `joint_state_publisher` (10 Hz) + `rws_commander`
      + `rws_trajectory_controller` + `gripper_action_server` + `move_group`

Optional: add `rviz:=true` to open RViz2.

### 2. EGM mode — full MoveIt2 stack at 250 Hz

EGM is provided by the companion package `yumi_egm_interface`:

```bash
ros2 launch yumi_egm_interface yumi_egm.launch.py robot_ip:=192.168.125.1
```

Starts: `robot_state_publisher` + `rws_commander` + `egm_trajectory_controller`
      + `gripper_action_server` + `move_group`

Automatically adds a table collision object to the planning scene 8 s after startup.

Optional arguments:
```bash
ros2 launch yumi_egm_interface yumi_egm.launch.py \
  robot_ip:=192.168.125.1 \
  left_udp_port:=6511 \
  right_udp_port:=6512 \
  rviz:=true
```

---

## Scripts

Located in `scripts/`. Run directly with Python (no `ros2 run` needed).

```bash
cd ~/yumi_ws/src/yumi_dissertation/yumi_rws_interface/scripts/
```

### Task sequence (requires ROS 2 + active launch)

```bash
# Execute the full RCD wire insertion cycle (loops until Ctrl+C)
python3 yumi_task_sequence.py
```

### Pose teaching

```bash
# Record all poses interactively (requires /joint_states to be publishing)
python3 record_poses.py

# Update a single pose without re-recording everything
python3 record_poses.py --update right_pick
```

Poses are saved to `scripts/poses.yaml`. Required poses for the task:
`right_pick_approach`, `right_pick`, `handover_right`, `handover_left`,
`left_insert_approach`, `left_insert`

### Diagnostics

```bash
# Discover available RWS endpoints on the controller
python3 test_rws_discovery.py
python3 test_rws_discovery.py 192.168.125.1
```

### Archived scripts (validated, kept for reference)

Located in `scripts/archive/`:

| Script | Description |
|--------|-------------|
| `replay_handover_safe.py` | First validated full-cycle dual-arm handover — hardcoded positions with intermediate safety waypoints |
| `teach_handover.py` | Interactive dual-arm handover teach & replay |
| `teach_pick_place.py` | Interactive single-arm pick & place teach |
| `replay_pick_place.py` | Single-arm pick & place replay |
| `move_yumi.py` | Send a single test joint target to the right arm |
| `move_yumi_home.py` | Move both arms to ABB calibration home |
| `move_yumi_wasd_no_rapid.py` | Live WASD keyboard control of the right arm (joints 1–3) |

---

## Useful Commands

```bash
# Build
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

### Joint Order Mapping (URDF ↔ RWS/EGM)

RWS and EGM return joints as `[rax_1, rax_2, rax_3, rax_4, rax_5, rax_6, eax_a]`.
The URDF expects `[joint_1, joint_2, joint_7, joint_3, joint_4, joint_5, joint_6]`
(joint 7 = gripper rail, sits between joint 2 and 3 in the URDF chain).

Reorder index: `RWS_TO_URDF_IDX = [0, 1, 6, 2, 3, 4, 5]`

This mapping is used in: `joint_state_publisher`, `rws_trajectory_controller`,
`egm_manager`, and `egm_trajectory_controller`.

### RWS Trajectory Strategy

`rws_trajectory_controller` receives full trajectories from MoveIt2 and executes
them via **`runMoveAbsJMulti`** — a RAPID routine that iterates over an array of
`jointtarget` waypoints.

**Why not send only the final point?**
MoveIt2 generates trajectories that avoid obstacles along the full path. Sending
only the final point causes the IRC5 to take a straight joint-space line, ignoring
the planned path and potentially causing collisions.

**How it works:**
1. The trajectory from MoveIt2 (hundreds of points) is downsampled to at most
   `max_waypoints` (default: 16, hard limit: 18) using a joint-step threshold
   (`resample_max_joint_step_deg`, default: 6°). Only points where any joint
   moved more than the threshold from the last kept point are retained.
   First and last points are always kept.
2. Each selected waypoint is written to the RAPID array `wp_array{i}` as a
   `jointtarget` value (`[[rax_1..rax_6],[eax_a,9E9,...]]`).
3. `wp_count`, `wp_speed`, and `routine_name_input = "runMoveAbsJMulti"` are written.
4. Signal `RUN_RAPID_ROUTINE` is pulsed to trigger RAPID execution.

**Safety features:**
- `normalize_ext_axis`: adjusts `eax_a` (joint 7) in all waypoints to take the
  shortest path, avoiding 360° wraps in the `both_arms` case.
- `max_jump_deg` (default: 220°): rejects the entire trajectory if any consecutive
  joint step exceeds the threshold, preventing unsafe moves from stale or corrupt
  waypoints.

### EGM Trajectory Strategy

`egm_trajectory_controller` follows trajectories in real time at 250 Hz (4 ms cycle)
by sending `EgmSensor` protobuf packets over UDP.

**Dual-arm timing constraint:** all sends must precede all receives in each 4 ms
cycle. Sequential per-arm send+receive increases cycle time to ~100 ms, exceeding
the EGM `comm_timeout` and stopping the robot.

**Session keep-alive:** hold threads continuously send the current position back to
the IRC5 between trajectories. If no UDP packet is received within the controller's
`comm_timeout` (typically 1–5 s), the EGM session stops.

### RAPID Trigger Mechanism

All motion commands use a "write variable → pulse signal" pattern:

**Multi-waypoint motion (`runMoveAbsJMulti`):**
1. Write `wp_array{1}` .. `wp_array{N}` — each a `jointtarget`
2. Write `wp_count`, `wp_speed`, `routine_name_input = "runMoveAbsJMulti"`
3. Pulse `RUN_RAPID_ROUTINE` high then low (delay: 0.1 s)

**Gripper commands:**
1. Write `command_input` (+ `target_position_input` for `g_MoveTo`)
2. Pulse `RUN_SG_ROUTINE` high then low

### Network Configuration

| Parameter | Value |
|-----------|-------|
| Robot IP | `192.168.125.1` |
| RWS Port | `80` (HTTP) |
| Authentication | HTTP Digest |
| Username | `Default User` |
| Password | `robotics` |
| EGM left arm UDP port | `6511` |
| EGM right arm UDP port | `6512` |
| RAPID Tasks | `T_ROB_L` (left), `T_ROB_R` (right) |
| Motion Module | `TRobRAPID` |
| Gripper Module | `TRobSG` |
