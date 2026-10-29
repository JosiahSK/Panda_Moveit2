# 🤖 MoveIt 2 + Panda Robot — Complete Teaching Repository

> **ROS 2 Jazzy Jalisco** | Branch: `jazzy`  
> For ROS 2 Humble (LTS), switch to the [`humble`](../../tree/humble) branch.

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-purple?logo=ros)](https://docs.ros.org/en/jazzy/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt-2.8.x-orange)](https://moveit.ros.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.12-yellow?logo=python)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C++-20-blue?logo=cplusplus)](https://isocpp.org/)

---

## 📚 About This Repository

This repository is a **comprehensive, hands-on course** for learning MoveIt 2 motion planning with the Franka Panda 7-DOF robot arm in RViz. It is structured for university students and robotics enthusiasts with:

- 📦 Ready-to-run ROS 2 packages (Python & C++)
- 🗺️ Visual architecture diagrams and motion planning flowcharts
- 📷 Annotated RViz screenshots for every exercise
- 🧩 Progressive lessons from installation to advanced pick & place
- 📊 Benchmark graphs of planning time vs. algorithms

---

## 🗂️ Repository Structure

```
panda_moveit2_tutorial/
├── README.md                          # You are here
├── docs/                              # 📖 Lesson files (Wiki-style)
│   ├── 00_overview.md                 # MoveIt 2 architecture overview
│   ├── 01_installation.md             # Installation guide (Humble)
│   ├── 02_rviz_basics.md              # RViz & MoveIt plugin tour
│   ├── 03_motion_planning_python.md   # Python motion planning tutorial
│   ├── 04_motion_planning_cpp.md      # C++ motion planning tutorial
│   ├── 05_constraints.md              # Joint & Cartesian constraints
│   ├── 06_pick_and_place.md           # Pick & place pipeline
│   ├── 07_custom_planners.md          # Custom planner integration
│   └── images/                        # Screenshots & diagrams
└── src/
    ├── panda_moveit2_tutorials/       # Python ROS 2 package
    │   ├── panda_moveit2_tutorials/   # Python nodes
    │   ├── scripts/                   # Standalone scripts
    │   ├── launch/                    # Launch files
    │   ├── config/                    # YAML configs
    │   ├── package.xml
    │   └── setup.py
    └── panda_moveit2_cpp/             # C++ ROS 2 package
        ├── src/                       # C++ source files
        ├── include/                   # C++ headers
        ├── CMakeLists.txt
        └── package.xml
```

---

## 🧠 MoveIt 2 Architecture Overview

```mermaid
flowchart TD
    User["👤 User / Script"]
    MoveGroupNode["move_group Node\n(MoveIt 2 Core)"]
    OMPL["OMPL Planner\n(RRTConnect, PRM, etc.)"]
    STOMP["STOMP / CHOMP\nTrajectory Optimization"]
    Kinematics["KDL / TRAC-IK\nInverse Kinematics"]
    CollisionCheck["FCL / Bullet\nCollision Checking"]
    PlanningScene["Planning Scene\nOctomap / Mesh obstacles"]
    ControllerIface["JointTrajectoryController\n(ros2_control)"]
    RViz["RViz 2\nMotionPlanning Plugin"]
    PandaRobot["🦾 Panda Robot\n(Simulation / Real)"]

    User -->|"MoveGroupInterface API"| MoveGroupNode
    RViz -->|"Interactive Marker Goals"| MoveGroupNode
    MoveGroupNode --> OMPL
    MoveGroupNode --> STOMP
    MoveGroupNode --> Kinematics
    MoveGroupNode --> CollisionCheck
    MoveGroupNode --> PlanningScene
    MoveGroupNode -->|"Trajectory\nexecution"| ControllerIface
    ControllerIface --> PandaRobot
    PandaRobot -->|"Joint States"| MoveGroupNode
    MoveGroupNode -->|"Visualization"| RViz
```

---

## 🏗️ Motion Planning Pipeline

```mermaid
flowchart LR
    Goal["🎯 Goal Pose\nor Joint Target"]
    Validate["Validate Goal\n+ Scene Check"]
    Plan["Planning Request\nOMPL / CHOMP / STOMP"]
    Traj["Raw Trajectory\n(Waypoints)"]
    PostProc["Post-Processing\nTime Parameterization"]
    Execute["Execute\nJoint Trajectory"]
    Monitor["Monitor Execution\nFeedback Loop"]
    Done["✅ Goal Reached"]

    Goal --> Validate
    Validate --> Plan
    Plan --> Traj
    Traj --> PostProc
    PostProc --> Execute
    Execute --> Monitor
    Monitor -->|"Success"| Done
    Monitor -->|"Preempt / Error"| Plan
```

---

## 🚀 5-Minute Student Sandbox (Try It Now!)

Want to test motion planning immediately before reading the code?

```bash
# 1. Launch Panda in RViz 2
ros2 launch moveit_resources_panda_moveit_config demo.launch.py

# 2. In RViz:
#    a. Locate the "MotionPlanning" panel on the left.
#    b. Change "Goal State" to "ready".
#    c. Click "Plan & Execute" — watch the Panda arm glide into position!
```

---

## ⚡ Quick Start

### Prerequisites
- Ubuntu 24.04 (or WSL2 with Ubuntu 24.04)
- ROS 2 Jazzy Jalisco (see [Installation Guide](docs/01_installation.md))

### 1. Install Dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-franka-ros2 \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers
```

### 2. Clone & Build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b jazzy https://github.com/JosiahSK/Panda_Moveit2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch Panda in RViz

In your first terminal, launch the MoveIt 2 demo environment:

```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

*RViz will open displaying the Franka Panda arm with the MotionPlanning display plugin loaded.*

---

## 🎮 Hands-On Task Catalog & Execution Guide

Open a **second terminal**, source your workspace, and run any of the tasks below:

```bash
source ~/ros2_ws/install/setup.bash
```

### 🐍 Python Tasks (`panda_moveit2_tutorials`)

| Task # | Command | What It Does & What to Watch For in RViz |
|---|---|---|
| **Task 1: Basic Joint Planning** | `ros2 run panda_moveit2_tutorials basic_motion_planner` | **Joint-space motion**: Commands the robot first to predefined `ready` pose, then directly sets individual joint radian angles (`panda_joint1` to `panda_joint7`). Watch the arm transition between joint states. |
| **Task 2: Cartesian Pose Target** | `ros2 run panda_moveit2_tutorials pose_goal_planner` | **Inverse Kinematics (IK)**: Sets a 6-DOF Cartesian target pose $(x=0.307, y=0.0, z=0.590)$ with gripper pointing down. MoveIt calculates the inverse kinematics and executes smooth trajectory. |
| **Task 3: Cartesian Inspection Path** | `ros2 run panda_moveit2_tutorials cartesian_path_planner` | **Continuous Waypoints**: Generates a straight-line rectangular inspection pattern $(20\text{cm} \times 20\text{cm})$ in front of the robot. Demonstrates multi-waypoint straight-line Cartesian execution. |
| **Task 4: Collision Avoidance Scene** | `ros2 run panda_moveit2_tutorials collision_avoidance_demo` | **Dynamic Obstacle Avoidance**: Spawns a table and a tall red wall obstacle directly in front of the arm. MoveIt dynamically plans an arc *over and around* the barrier without touching it, then cleans up the scene. |
| **Task 5: Path Constraints** | `ros2 run panda_moveit2_tutorials constrained_motion_demo` | **Carrying Liquid Demo**: Enforces an `OrientationConstraint` keeping the gripper strictly vertical ($\pm 0.25\text{ rad}$) throughout a long sideways trajectory. Perfect for showing students how to carry a full glass of water. |
| **Task 6: Pick & Place Pipeline** | `ros2 run panda_moveit2_tutorials pick_and_place` | **Full 9-Stage Grasp Pipeline**: Approaches object from above $\to$ opens gripper $\to$ descends $\to$ grasps $\to$ lifts $\to$ translates $\to$ places on destination platform $\to$ opens fingers $\to$ returns home. |
| **Task 7: Planner Benchmarking** | `ros2 run panda_moveit2_tutorials planner_benchmark` | **Algorithm Comparison**: Benchmarks `RRTConnect`, `PRM`, and `LBKPIECE` over 5 runs each. Prints real-time planning times and success rate statistics directly to terminal. |

---

### ⚡ C++ Tasks (`panda_moveit2_cpp`)

High-performance native C++ implementations using `moveit::planning_interface::MoveGroupInterface`:

| Task # | Command | Explanation |
|---|---|---|
| **Task 8: C++ Joint Planner** | `ros2 run panda_moveit2_cpp basic_motion_planner_cpp` | Initializes a multi-threaded executor node, sets joint value vector targets, and plans with max velocity/acceleration scaling. |
| **Task 9: C++ Pose Goal Planner** | `ros2 run panda_moveit2_cpp pose_planner_cpp` | Sets 6-DOF position and quaternion orientation via `setPoseTarget()` and monitors execution status. |
| **Task 10: C++ Cartesian Waypoints** | `ros2 run panda_moveit2_cpp cartesian_planner_cpp` | Uses `computeCartesianPath()` with a $1\text{cm}$ interpolation step resolution and jump threshold validation. |
| **Task 11: C++ Planning Scene Manager** | `ros2 run panda_moveit2_cpp scene_manager_cpp` | Programmatically adds primitive collision objects (`shape_msgs/SolidPrimitive::BOX`) to the world, plans around them, and cleans them up. |

---

## 📖 Lesson Index

| # | Lesson | Topic | Difficulty |
|---|--------|-------|------------|
| 00 | [Overview](docs/00_overview.md) | MoveIt 2 architecture, concepts | 🟢 Beginner |
| 01 | [Installation](docs/01_installation.md) | Full setup guide for Humble | 🟢 Beginner |
| 02 | [RViz Basics](docs/02_rviz_basics.md) | MoveIt plugin, interactive markers | 🟢 Beginner |
| 03 | [Python Planning](docs/03_motion_planning_python.md) | MoveGroupInterface in Python | 🟡 Intermediate |
| 04 | [C++ Planning](docs/04_motion_planning_cpp.md) | MoveGroupInterface in C++ | 🟡 Intermediate |
| 05 | [Constraints](docs/05_constraints.md) | Joint & path constraints | 🟡 Intermediate |
| 06 | [Pick & Place](docs/06_pick_and_place.md) | Full grasp pipeline | 🔴 Advanced |
| 07 | [Custom Planners](docs/07_custom_planners.md) | Swap planners, tune parameters | 🔴 Advanced |
| 08 | [Kinematics & Redundancy](docs/08_kinematics_and_redundancy.md) | FK, IK, coordinate frames & null-space | 🔴 Advanced |
| 📋 | [Repository Audit](docs/REPOSITORY_AUDIT.md) | Technical audit & QA verification report | 🔍 Engineering |

---

## 🔀 Branch Comparison

| Feature | `humble` branch | `jazzy` branch |
|---------|----------------|----------------|
| ROS 2 Version | Humble Hawksbill (LTS) | **Jazzy Jalisco** ← **(you are here)** |
| Ubuntu | 22.04 LTS | **24.04 LTS** |
| MoveIt 2 | 2.5.x | **2.8.x** |
| Python | 3.10 | **3.12** |
| Key Differences | Classic `moveit_resources` | **New `moveit_configs_utils` API** |
| Status | ✅ Supported until May 2027 | ✅ **Active Development** |

---

## 🤝 Contributing

Pull requests are welcome! Please open an issue first to discuss what you'd like to change.

## 📜 License

MIT License — see [LICENSE](LICENSE)

## 🙏 Acknowledgements

- [MoveIt 2 Official Documentation](https://moveit.ros.org/)
- [Franka Robotics](https://www.franka.de/)
- [JosiahSK/Panda_Moveit2](https://github.com/JosiahSK/Panda_Moveit2) — reference repo
- [Open Source Robotics Foundation](https://www.openrobotics.org/)
