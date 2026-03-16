# ABB YuMi MoveIt2 Configuration for ROS2 Jazzy

Complete MoveIt2 configuration for the ABB YuMi IRB 14000 dual-arm collaborative robot, tested on ROS2 Jazzy.

![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![License](https://img.shields.io/badge/License-BSD--3--Clause-green)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2024.04-orange)

## Overview

This repository provides a complete, production-ready MoveIt2 configuration for the ABB YuMi (IRB 14000) dual-arm collaborative robot. It includes:

- **Complete MoveIt2 integration** with OMPL motion planning
- **Dual-arm motion planning** with three planning groups: `left_arm`, `right_arm`, `both_arms`
- **Custom RViz configuration** with pre-configured MotionPlanning plugin
- **Comprehensive collision matrix** for safe dual-arm operation
- **Fake controllers** for simulation and testing
- **Pre-configured home positions** for reliable initialization

## Features

### Motion Planning
- ✅ OMPL planning pipeline with multiple algorithms (RRTConnect, RRTstar, PRM, etc.)
- ✅ KDL kinematics solver for each arm (7-DOF)
- ✅ Time-optimal trajectory parameterization
- ✅ Collision-aware planning with extensive self-collision avoidance
- ✅ Individual and coordinated dual-arm planning

### Robot Configuration
- ✅ Complete URDF with accurate kinematics
- ✅ Semantic robot description (SRDF) with planning groups
- ✅ Joint limits and velocity constraints
- ✅ Properly configured end effectors (grippers)
- ✅ Fixed frame: `world` → `yumi_base_link`

### Simulation & Visualization
- ✅ Fake trajectory controllers for testing without hardware
- ✅ Custom RViz config optimized for dual-arm manipulation
- ✅ Real-time trajectory visualization
- ✅ Interactive marker-based pose planning

## Prerequisites

- **ROS2 Jazzy** (Ubuntu 24.04 recommended)
- **MoveIt2** (jazzy distribution)
- **colcon** build tools

## Installation

### 1. Create workspace and clone

```bash
mkdir -p ~/yumi_ws/src
cd ~/yumi_ws/src

# Clone this repository
git clone https://github.com/YOUR_USERNAME/yumi_moveit2.git

# Clone YuMi description (upstream)
git clone https://github.com/kth-ros-pkg/yumi.git
```

### 2. Install dependencies

```bash
cd ~/yumi_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### Launch MoveIt Demo

```bash
ros2 launch yumi_moveit_config demo.launch.py
```

This launches:
- MoveGroup node with OMPL planning
- RViz with MotionPlanning plugin
- Fake trajectory controllers
- Robot state publisher

### Using RViz for Motion Planning

1. **Select Planning Group**
   - In RViz MotionPlanning panel, choose:
     - `left_arm` - Control left arm only
     - `right_arm` - Control right arm only
     - `both_arms` - Coordinated dual-arm control

2. **Plan Motion**
   - Drag interactive markers to set goal pose
   - Click "Plan" to generate trajectory
   - Click "Execute" to run on fake controllers
   - Click "Plan & Execute" for one-step operation

3. **Change Planning Algorithm**
   - In MotionPlanning panel → Planning tab
   - Select from: RRTConnect, RRTstar, PRM, TRRT, etc.

## Package Structure

```
yumi_moveit2/
├── yumi_description/          # Robot model (from upstream)
│   ├── urdf/
│   │   └── yumi.urdf         # Main robot description
│   └── meshes/               # Visual and collision meshes
│
└── yumi_moveit_config/        # MoveIt configuration
    ├── config/
    │   ├── yumi.srdf                  # Semantic robot description
    │   ├── kinematics.yaml            # KDL solver config
    │   ├── joint_limits.yaml          # Joint constraints
    │   ├── moveit_controllers.yaml    # Controller config
    │   ├── ompl_planning.yaml         # OMPL planner config
    │   └── moveit.rviz                # Custom RViz config
    │
    ├── launch/
    │   └── demo.launch.py             # Main demo launcher
    │
    ├── package.xml
    └── CMakeLists.txt
```

## Configuration Details

### Planning Groups

The configuration defines three planning groups:

**1. left_arm** (7-DOF)
- Joints: `yumi_joint_1_l` through `yumi_joint_7_l`
- End effector: `gripper_l_base`
- KDL kinematics solver

**2. right_arm** (7-DOF)
- Joints: `yumi_joint_1_r` through `yumi_joint_7_r`
- End effector: `gripper_r_base`
- KDL kinematics solver

**3. both_arms** (14-DOF)
- Includes both left and right arm groups
- No IK solver (plan each arm separately)
- Coordinated motion planning

### Home Positions

Pre-configured safe home positions for each arm:

```yaml
left_arm home:
  joint_1_l: 0.0
  joint_2_l: -2.2689 rad (-130°)
  joint_3_l: 0.0
  joint_4_l: 0.5236 rad (30°)
  joint_5_l: 0.0
  joint_6_l: 0.6981 rad (40°)
  joint_7_l: 2.3562 rad (135°)

right_arm home:
  joint_1_r: 0.0
  joint_2_r: -2.2689 rad (-130°)
  joint_3_r: 0.0
  joint_4_r: 0.5236 rad (30°)
  joint_5_r: 0.0
  joint_6_r: 0.6981 rad (40°)
  joint_7_r: -2.3562 rad (-135°)
```

### Collision Avoidance

Comprehensive collision matrix with:
- **Adjacent link pairs** disabled (parent-child joints)
- **Gripper internal collisions** disabled (fingers, base)
- **Cross-arm collisions** carefully configured to prevent false positives
- **Self-collision checking** enabled for workspace validation

Key disabled collision pairs:
- All gripper internal parts (base ↔ fingers)
- Cross-arm gripper interactions (never physically possible)
- Remote link pairs that cannot collide

## Supported OMPL Planners

Available planning algorithms:

| Planner | Type | Best For |
|---------|------|----------|
| **RRTConnect** | Sampling | General purpose (default) |
| **RRTstar** | Optimal | Path optimization |
| **PRM** | Roadmap | Multi-query scenarios |
| **PRMstar** | Optimal roadmap | Quality multi-query |
| **TRRT** | Transition-based | Cost-aware planning |
| **SBL** | Bidirectional | Simple scenarios |
| **EST** | Sampling | Exploration |

Configure in RViz or programmatically in motion planning requests.

## Customization

### Modify Joint Limits

Edit `config/joint_limits.yaml`:

```yaml
joint_limits:
  yumi_joint_1_l:
    has_velocity_limits: true
    max_velocity: 2.618  # rad/s
    has_acceleration_limits: false
```

### Adjust Planning Parameters

Edit `config/ompl_planning.yaml`:

```yaml
left_arm:
  planner_configs:
    - RRTConnectkConfigDefault
  longest_valid_segment_fraction: 0.005  # Motion discretization
```

### Change Kinematics Solver

Edit `config/kinematics.yaml`:

```yaml
left_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

## Integration with Real Hardware

To use with real YuMi hardware:

1. Replace fake controllers with real hardware interface
2. Update `config/moveit_controllers.yaml`
3. Configure proper controller action servers
4. Ensure proper EtherCAT or network connection

Refer to [ABB YuMi ROS2 driver documentation](https://github.com/kth-ros-pkg/yumi) for hardware setup.

## Troubleshooting

### Issue: "Planning plugin not loaded"

**Solution:** Ensure OMPL planning library is installed:
```bash
sudo apt install ros-jazzy-moveit-planners-ompl
```

### Issue: "Collision detected in start state"

**Solution:** Robot may be starting in self-collision. Check:
1. Home position is valid
2. Collision matrix properly configured
3. Joint states are being published

### Issue: "Motion plan validation failed"

**Solution:**
1. Check collision matrix in SRDF
2. Verify trajectory doesn't pass through singularities
3. Adjust `longest_valid_segment_fraction` for finer discretization

### Issue: RViz not loading properly

**Solution:**
```bash
# Clear RViz config cache
rm -rf ~/.rviz2/
# Relaunch
ros2 launch yumi_moveit_config demo.launch.py
```

## Performance Tips

1. **Planning Time**
   - RRTConnect: Fast, good for most cases
   - RRTstar: Slower but better paths
   - Adjust timeout in planning requests

2. **Trajectory Quality**
   - Enable time parameterization (already configured)
   - Increase planning attempts for better solutions
   - Use smoothing post-processing

3. **Dual-Arm Coordination**
   - Plan each arm separately then validate
   - Use `both_arms` group for coordinated constraints
   - Consider sequential planning for complex tasks

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Test thoroughly with demo launch
4. Submit pull request with clear description

## License

BSD-3-Clause License

Copyright (c) 2024

## Acknowledgments

- **ABB** for the YuMi robot platform
- **KTH Royal Institute of Technology** for upstream YuMi ROS packages
- **MoveIt2 community** for the excellent motion planning framework

## References

- [ABB YuMi Product Page](https://new.abb.com/products/robotics/industrial-robots/yumi)
- [MoveIt2 Documentation](https://moveit.ros.org/)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [OMPL Documentation](https://ompl.kavrakilab.org/)

## Support

For issues and questions:
- GitHub Issues: [Report a bug](https://github.com/YOUR_USERNAME/yumi_moveit2/issues)
- MoveIt Discourse: [Ask the community](https://discourse.ros.org/c/moveit)

---

**Tested on:** ROS2 Jazzy, Ubuntu 24.04 LTS
**Last Updated:** December 2024
