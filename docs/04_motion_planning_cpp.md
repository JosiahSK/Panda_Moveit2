# Lesson 04 — C++ Motion Planning with MoveIt 2

> **Branch:** `jazzy` | ROS 2 Jazzy Jalisco

---

## Overview

This lesson covers MoveIt 2 motion planning via the **C++ MoveGroupInterface** API. C++ gives you:
- Lower latency
- Direct access to the MoveIt 2 internal APIs
- Better integration with `rclcpp` lifecycle nodes

## Setup & C++ Tasks Quick Reference

Make sure RViz is running:

```bash
# Terminal 1: Launch Panda in RViz
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

Build and run C++ executables in Terminal 2:

```bash
cd ~/ros2_ws
colcon build --packages-select panda_moveit2_cpp
source install/setup.bash
```

### 📋 C++ Tasks Quick Reference

| Command | Task Description | Key C++ Classes Used |
|---|---|---|
| `ros2 run panda_moveit2_cpp basic_motion_planner_cpp` | Joint Target Planning | `MoveGroupInterface::setNamedTarget()`, `setJointValueTarget()` |
| `ros2 run panda_moveit2_cpp pose_planner_cpp` | Cartesian Pose Planning | `MoveGroupInterface::setPoseTarget()`, `geometry_msgs::msg::Pose` |
| `ros2 run panda_moveit2_cpp cartesian_planner_cpp` | Cartesian Waypoint Interpolation | `MoveGroupInterface::computeCartesianPath()` |
| `ros2 run panda_moveit2_cpp scene_manager_cpp` | Collision Objects in Scene | `PlanningSceneInterface::applyCollisionObject()` |

---

**File:** [`src/panda_moveit2_cpp/src/basic_motion_planner.cpp`](../src/panda_moveit2_cpp/src/basic_motion_planner.cpp)

```cpp
/**
 * Lesson 04 - Example 1: Basic joint-space motion planning in C++
 * Uses MoveGroupInterface to plan and execute to a named joint state.
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("panda_motion_planner");
static const std::string PLANNING_GROUP = "panda_arm";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "panda_motion_planner",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Spin the node in a separate thread so MoveGroupInterface can work
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // ─── Create MoveGroupInterface ────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Print useful info
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group.getEndEffectorLink().c_str());

    // ─── Plan to named state 'ready' ─────────────────────────────────
    move_group.setNamedTarget("ready");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    RCLCPP_INFO(LOGGER, "Planning to 'ready': %s", success ? "SUCCESS" : "FAILED");

    if (success)
    {
        RCLCPP_INFO(LOGGER, "Executing trajectory with %zu points...",
                    plan.trajectory_.joint_trajectory.points.size());
        move_group.execute(plan);
    }

    // ─── Plan to custom joint values ──────────────────────────────────
    std::vector<double> joint_group_positions(7);
    joint_group_positions[0] =  0.0;          // joint1
    joint_group_positions[1] = -M_PI / 4.0;   // joint2 = -45°
    joint_group_positions[2] =  0.0;           // joint3
    joint_group_positions[3] = -M_PI / 2.0;   // joint4 = -90°
    joint_group_positions[4] =  0.0;           // joint5
    joint_group_positions[5] =  M_PI / 3.0;   // joint6 = 60°
    joint_group_positions[6] =  M_PI / 4.0;   // joint7 = 45°

    move_group.setJointValueTarget(joint_group_positions);
    
    success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Custom joint plan: %s", success ? "SUCCESS" : "FAILED");
    
    if (success)
        move_group.execute(plan);

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
```

---

## Example 2: Cartesian Pose Goal (C++)

```cpp
/**
 * Lesson 04 - Example 2: Plan to a Cartesian end-effector pose.
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pose_planner");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("panda_pose_planner");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
    
    // Set velocity/acceleration scaling
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(5.0);

    // ─── Define target pose ──────────────────────────────────────────
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.307;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.590;
    target_pose.orientation.x = 0.924;
    target_pose.orientation.y = -0.383;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    RCLCPP_INFO(LOGGER, "Pose planning: %s (%.2f seconds)",
                success ? "SUCCESS" : "FAILED",
                plan.planning_time_);

    if (success)
        move_group.execute(plan);

    move_group.clearPoseTargets();
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
```

---

## Example 3: Cartesian Path in C++

```cpp
/**
 * Lesson 04 - Example 3: Compute a Cartesian waypoint path.
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("cartesian_planner");

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread t([&exec]() { exec.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
    
    // Current pose as starting waypoint
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
    
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);
    
    // Move 15cm in X
    geometry_msgs::msg::Pose p2 = start_pose;
    p2.position.x += 0.15;
    waypoints.push_back(p2);
    
    // Move 10cm in Z (up)
    geometry_msgs::msg::Pose p3 = p2;
    p3.position.z += 0.10;
    waypoints.push_back(p3);
    
    // Move back in X
    geometry_msgs::msg::Pose p4 = p3;
    p4.position.x -= 0.15;
    waypoints.push_back(p4);
    
    // Compute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;        // 1cm step
    const double jump_threshold = 0.0;   // Disable jump detection
    
    double fraction = move_group.computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory
    );
    
    RCLCPP_INFO(rclcpp::get_logger("cartesian"), 
                "Cartesian path: %.1f%% achievable", fraction * 100.0);
    
    if (fraction > 0.9)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group.execute(plan);
    }

    rclcpp::shutdown();
    t.join();
    return 0;
}
```

---

## Example 4: Collision Objects in C++

```cpp
/**
 * Lesson 04 - Example 4: Add/remove collision objects from the planning scene.
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("scene_manager_cpp");

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread t([&exec]() { exec.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
    moveit::planning_interface::PlanningSceneInterface psi;

    // ─── Add collision box ────────────────────────────────────────────
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "obstacle_box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.1, 0.4, 0.4};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.2;
    box_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    psi.applyCollisionObject(collision_object);
    RCLCPP_INFO(rclcpp::get_logger("scene"), "Added obstacle box.");
    
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Plan around the obstacle
    move_group.setNamedTarget("extended");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("scene"), "Found collision-free path!");
        move_group.execute(plan);
    }

    // Remove obstacle
    std::vector<std::string> ids = {"obstacle_box"};
    psi.removeCollisionObjects(ids);
    RCLCPP_INFO(rclcpp::get_logger("scene"), "Removed obstacle box.");

    rclcpp::shutdown();
    t.join();
    return 0;
}
```

---

## CMakeLists.txt for C++ Package

```cmake
cmake_minimum_required(VERSION 3.8)
project(panda_moveit2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

set(DEPS
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_core
  geometry_msgs
  moveit_msgs
  shape_msgs
)

# Basic motion planner executable
add_executable(basic_motion_planner_cpp src/basic_motion_planner.cpp)
ament_target_dependencies(basic_motion_planner_cpp ${DEPS})

# Pose planner executable
add_executable(pose_planner_cpp src/pose_planner.cpp)
ament_target_dependencies(pose_planner_cpp ${DEPS})

# Cartesian planner executable
add_executable(cartesian_planner_cpp src/cartesian_planner.cpp)
ament_target_dependencies(cartesian_planner_cpp ${DEPS})

# Scene manager executable
add_executable(scene_manager_cpp src/scene_manager.cpp)
ament_target_dependencies(scene_manager_cpp ${DEPS})

install(TARGETS
  basic_motion_planner_cpp
  pose_planner_cpp
  cartesian_planner_cpp
  scene_manager_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## Build & Run

```bash
cd ~/ros2_ws
colcon build --packages-select panda_moveit2_cpp --symlink-install
source install/setup.bash

# Run any example
ros2 run panda_moveit2_cpp basic_motion_planner_cpp
ros2 run panda_moveit2_cpp pose_planner_cpp
ros2 run panda_moveit2_cpp cartesian_planner_cpp
ros2 run panda_moveit2_cpp scene_manager_cpp
```

---

## Python vs C++ Comparison

| Aspect | Python | C++ |
|--------|--------|-----|
| Ease of use | ✅ Easier | ❌ More verbose |
| Runtime performance | ❌ Slower startup | ✅ Faster |
| IDE support | ✅ Great | ✅ Great |
| MoveIt API completeness | ✅ Full bindings | ✅ Full native |
| Real-time control | ❌ Not suitable | ✅ Suitable |
| Rapid prototyping | ✅ Best | ❌ Slower iteration |

---

## Next Steps

👉 Proceed to [Lesson 05 — Constraints](05_constraints.md)
