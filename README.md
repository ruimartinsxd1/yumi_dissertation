# YuMi Dissertation — Collaborative Robotics for RCD Pre-Assembly

Master's dissertation — FEUP/INESCTEC 2026
ABB YuMi IRB 14000 | ROS 2 Jazzy | MoveIt2 | RWS | EGM

**Author:** Rui Martins (up202108756@edu.fe.up.pt // rui.m.martins@inesctec.pt)

---

## Repository Structure

| Package | Type | Description |
|---------|------|-------------|
| `yumi_rws_interface/` | Original | ROS 2 interface via RWS (HTTP, 10 Hz) — joint states, RAPID control, trajectory execution, gripper control |
| `yumi_egm_interface/` | Original | ROS 2 interface via EGM (UDP, 250 Hz) — real-time trajectory execution |
| `yumi_ros2/` | Submodule (fork) | Fork of [Jshulgach/yumi_ros2](https://github.com/Jshulgach/yumi_ros2) — URDF model and MoveIt2 config, adapted for ROS 2 Jazzy + EGM |

---

## Cloning

This repository uses a git submodule (`yumi_ros2`). Clone with:

```bash
git clone --recurse-submodules https://github.com/ruimartinsxd1/yumi_dissertation.git
```

If you already cloned without `--recurse-submodules`, run:

```bash
git submodule update --init --recursive
```

---

## Quick Start

### Build

```bash
cd ~/yumi_ws
colcon build --symlink-install
source install/setup.bash
```

Or build only the interface packages:

```bash
colcon build --packages-select yumi_rws_interface yumi_egm_interface --symlink-install
```

### Launch — RWS mode (10 Hz, RAPID-based)

```bash
ros2 launch yumi_rws_interface yumi_rws.launch.py robot_ip:=192.168.125.1
```

### Launch — EGM mode (250 Hz, real-time)

```bash
ros2 launch yumi_egm_interface yumi_egm.launch.py robot_ip:=192.168.125.1
```

With RViz:

```bash
unset GTK_PATH
source ~/yumi_ws/install/setup.bash
ros2 launch yumi_egm_interface yumi_egm.launch.py rviz:=true
```

### Run the task sequence

```bash
python3 src/yumi_dissertation/yumi_rws_interface/scripts/yumi_task_sequence.py
```

---

## Robot Prerequisites

1. YuMi connected to the PC — default IP `192.168.125.1`
2. FlexPendant in **AUTO** mode
3. RAPID modules `TRobRAPID` and `TRobSG` loaded in both tasks (`T_ROB_L`, `T_ROB_R`)
4. RAPID running (`PP to Main` + `Play`)

---

## Package Overview

### yumi_rws_interface — RWS (HTTP, 10 Hz)

Nodes: `joint_state_publisher`, `rws_commander`, `rws_trajectory_controller`, `gripper_action_server`, `gripper_service`

Used in RWS mode and also provides the gripper and commander nodes for EGM mode.
See [`yumi_rws_interface/README.md`](yumi_rws_interface/README.md) for full documentation.

### yumi_egm_interface — EGM (UDP, 250 Hz)

Nodes: `egm_trajectory_controller` (also publishes `/joint_states` at 250 Hz)

Depends on `yumi_rws_interface` for gripper control and RWS signals (EGM start/stop).

### yumi_ros2 — URDF + MoveIt2 config (submodule)

Fork of [Jshulgach/yumi_ros2](https://github.com/Jshulgach/yumi_ros2) at
[github.com/ruimartinsxd1/yumi_ros2](https://github.com/ruimartinsxd1/yumi_ros2).

Modifications from upstream:
- `yumi_moveit_config`: new SRDF with `both_arms` planning group, `moveit_controllers_egm.yaml`, `joint_limits.yaml`, updated launch for MoveIt2 Jazzy
- `yumi_description`: minor mesh fix (`body_without_cam.stl` to remove Zivid camera from RViz)
