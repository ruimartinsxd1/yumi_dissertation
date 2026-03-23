# yumi_egm_interface

ROS 2 interface for controlling the ABB YuMi IRB 14000 dual-arm robot via
EGM (Externally Guided Motion) — real-time UDP at 250 Hz.
Developed as part of the dissertation **"Collaborative Robotics for RCD Pre-Assembly"**
at FEUP/INESCTEC, 2026.

**Author:** Rui Martins (up202108756@edu.fe.up.pt)
**Supervisors:** Prof. Luís F. Rocha and Carlos M Costa

> For HTTP-based RWS control (10 Hz), gripper, and RAPID services, see the companion package
> [`yumi_rws_interface`](../yumi_rws_interface/).

---

## Package Description

This package provides the EGM (UDP) half of the ROS 2 ↔ ABB YuMi bridge:

- Executes MoveIt2 trajectories in real time at 250 Hz (4 ms EGM cycle)
- Publishes `/joint_states` at 250 Hz from EGM feedback (replaces the 10 Hz RWS publisher)
- Exposes `FollowJointTrajectory` action servers for left arm, right arm, and both arms
- Manages dual UDP channels (port 6511 left, 6512 right) with background keep-alive threads

Depends on `yumi_rws_interface` for:
- Gripper control (`gripper_action_server` node)
- RAPID commander (`rws_commander` node)
- EGM start/stop signals via RWS

---

## Nodes

| Node | Entry point | Description |
|------|------------|-------------|
| `egm_trajectory_controller` | `egm_trajectory_controller` | FollowJointTrajectory action server + `/joint_states` publisher at 250 Hz |

---

## Launch

### Full EGM stack (robot + MoveIt2)

```bash
ros2 launch yumi_egm_interface yumi_egm.launch.py robot_ip:=192.168.125.1
```

With RViz:

```bash
ros2 launch yumi_egm_interface yumi_egm.launch.py robot_ip:=192.168.125.1 rviz:=true
```

Optional arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_ip` | `192.168.125.1` | IRC5 controller IP |
| `left_udp_port` | `6511` | UDP port for left arm EGM |
| `right_udp_port` | `6512` | UDP port for right arm EGM |
| `rviz` | `false` | Launch RViz2 |

---

## Build

```bash
cd ~/yumi_ws
colcon build --packages-select yumi_egm_interface --symlink-install
source install/setup.bash
```

---

## Technical Notes

### EGM Cycle (4 ms)

The controller sends/receives protobuf packets over UDP at 4 ms intervals.
**Critical:** all sends must precede all receives in each cycle — sequential
per-arm send+receive increases latency to ~100 ms and exceeds the IRC5 `comm_timeout`.

### Session Keep-Alive

Background threads continuously send the current position back to the IRC5
between trajectories, preventing the EGM session from timing out.

### Joint Order

EGM returns `[rax_1..rax_6, eax_a]`. URDF expects `[j1, j2, j7, j3, j4, j5, j6]`.
Conversion index: `[0, 1, 6, 2, 3, 4, 5]`.

### Network

| Parameter | Value |
|-----------|-------|
| Robot IP | `192.168.125.1` |
| EGM left port | `6511` (UDP) |
| EGM right port | `6512` (UDP) |
| RWS port | `80` (HTTP, used for EGM signals) |
