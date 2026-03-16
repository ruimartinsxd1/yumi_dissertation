# YuMi Dissertation — Collaborative Robotics for RCD Pre-Assembly

Master's dissertation — FEUP/INESCTEC 2026
ABB YuMi IRB 14000 | ROS 2 Jazzy | MoveIt2 | RWS | EGM

**Author:** Rui Martins (up202102606@up.pt // rui.m.martins@inesctec.pt)

---

## Packages

| Package | Description |
|---------|-------------|
| `yumi_rws_interface/` | ROS 2 interface for ABB YuMi — RWS + EGM control, MoveIt2 integration, task scripts. Written from scratch. |
| `yumi_ros2/` | Fork of [Jshulgach/yumi_ros2](https://github.com/Jshulgach/yumi_ros2). URDF model and MoveIt2 config, adapted for ROS 2 Jazzy + EGM + real hardware. |

---

## Quick Start

### Build

```bash
cd ~/yumi_ws
colcon build --symlink-install
source install/setup.bash
```

Or build only the custom package:

```bash
colcon build --packages-select yumi_rws_interface --symlink-install
```

### Launch (RWS mode — 10 Hz, RAPID-based)

```bash
ros2 launch yumi_rws_interface yumi_rws.launch.py robot_ip:=192.168.125.1
```

### Launch (EGM mode — 250 Hz, real-time)

```bash
ros2 launch yumi_rws_interface yumi_egm.launch.py robot_ip:=192.168.125.1
```

```bash
unset GTK_PATH
source ~/yumi_ws/install/setup.bash
ros2 launch yumi_rws_interface yumi_egm.launch.py rviz:=true
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
