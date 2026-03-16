# yumi_ros2

Fork of [Jshulgach/yumi_ros2](https://github.com/Jshulgach/yumi_ros2) (itself derived from
[KTH/yumi](https://github.com/kth-ros-pkg/yumi)), adapted for ROS 2 Jazzy and real-hardware
use with EGM. Part of the dissertation **"Collaborative Robotics for RCD Pre-Assembly"**,
FEUP/INESCTEC 2026.

---

## Packages

### `yumi_description/`

URDF model and meshes for the ABB YuMi IRB 14000. Kept largely as-is from upstream.
The pre-compiled `yumi.urdf` is checked in directly so launch files can reference it
without running xacro at launch time.

**Note on joint numbering (ABB convention):**
The physical joint order per arm is 1, 2, 7, 3, 4, 5, 6 — joint 7 (the prismatic
gripper rail, `eax_a`) sits between joints 2 and 3 in the URDF kinematic chain.

### `yumi_moveit_config/`

MoveIt2 configuration for ROS 2 Jazzy. Significantly modified from upstream for
real-hardware and EGM use.

---

## Changes from Upstream

### `config/yumi.srdf` — rewritten

- Three planning groups: `left_arm` (7-DOF), `right_arm` (7-DOF), `both_arms` (14-DOF composite)
- Two gripper groups: `left_gripper`, `right_gripper` (single prismatic joint each)
- Home states for all five groups with accurate joint values matching the physical robot
- `end_effector` declarations linking each gripper to its parent arm
- `virtual_joint` fixing the robot to the `world` frame at `yumi_base_link`
- 33 disabled collision pairs: adjacent links, gripper internal pairs, and cross-arm
  pairs that MoveIt2 falsely flags at the handover configuration

### `config/moveit_controllers.yaml` — rewritten

- Added `both_arms_controller` (14 joints)
- Gripper action namespace changed from `gripper_action` to `gripper_cmd`
- Gripper controlled joint changed to the single prismatic `gripper_{l|r}_joint`

### `config/moveit_controllers_egm.yaml` — new file

Same structure as `moveit_controllers.yaml`, used by `yumi_egm.launch.py`. Kept
separate so EGM-specific controller timing parameters can be added without affecting
the RWS launch.

### `config/joint_limits.yaml` — new file (not in upstream)

Velocity and acceleration limits for all 16 joints matching ABB YuMi specifications:
- Joints 1–3 and 7: 3.1416 rad/s
- Joints 4–6: 6.9813 rad/s
- Gripper joints: 0.025 m/s

### `launch/demo.launch.py` — rewritten

Uses `MoveItConfigsBuilder` (modern Jazzy API). Loads the joint limits file.
Uses fake controllers — suitable for simulation and offline planning only.

---

## Quick Start (simulation only)

```bash
cd ~/yumi_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch yumi_moveit_config demo.launch.py
```

For real-hardware use, see `../yumi_rws_interface/README.md`.

---

## Planning Groups

| Group | DOF | IK Solver |
|-------|-----|-----------|
| `left_arm` | 7 | KDL |
| `right_arm` | 7 | KDL |
| `both_arms` | 14 | None (plan each arm separately) |
| `left_gripper` | 1 | — |
| `right_gripper` | 1 | — |

The `both_arms` group has no IK solver — Cartesian-space goals must be set per arm.

---

## License

BSD-3-Clause (upstream). Modifications copyright 2026 Rui Martins.
