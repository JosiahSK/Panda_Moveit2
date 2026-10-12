# Lesson 03 — Python Motion Planning with MoveIt 2

> **Branch:** `humble` | ROS 2 Humble Hawksbill

---

## Overview

In this lesson, we use the **MoveIt 2 Python Bindings** (`moveit_py`) to:
1. Connect to a running `move_group` node
2. Plan to a joint target
3. Plan to a Cartesian pose target
4. Plan with waypoints (Cartesian path)
5. Add/remove collision objects programmatically

---

## Setup

Make sure the MoveIt demo is running in a separate terminal:

```bash
# Terminal 1: Start MoveIt + RViz
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

---

## Example 1: Basic Joint-Space Planning

**File:** [`src/panda_moveit2_tutorials/panda_moveit2_tutorials/basic_motion_planner.py`](../src/panda_moveit2_tutorials/panda_moveit2_tutorials/basic_motion_planner.py)

```python
#!/usr/bin/env python3
"""
Lesson 03 - Example 1: Plan to a named joint target.
Demonstrates MoveGroupInterface usage with MoveItPy.
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import math


def main():
    rclpy.init()
    
    # 1) Create the MoveItPy instance (connects to move_group node)
    moveit = MoveItPy(node_name="panda_motion_planner")
    
    # 2) Get the planning component for the panda_arm group
    panda_arm = moveit.get_planning_component("panda_arm")
    
    print("[INFO] MoveIt 2 Python node ready.")
    
    # ─── Example A: Plan to a named state ───────────────────────────
    print("[INFO] Planning to 'ready' named state...")
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    
    plan_result = panda_arm.plan()
    
    if plan_result:
        print("[INFO] Plan successful! Executing...")
        robot = moveit.get_robot()
        robot.execute(plan_result.trajectory, controllers=[])
    else:
        print("[WARN] Planning failed!")
    
    # ─── Example B: Plan to specific joint values ────────────────────
    print("[INFO] Planning to custom joint values...")
    
    robot_model = moveit.get_robot_model()
    robot_state = RobotState(robot_model)
    
    # Set custom joint configuration (in radians)
    joint_values = {
        "panda_joint1": 0.0,
        "panda_joint2": -math.pi / 4,   # -45 degrees
        "panda_joint3": 0.0,
        "panda_joint4": -math.pi / 2,   # -90 degrees
        "panda_joint5": 0.0,
        "panda_joint6":  math.pi / 3,   #  60 degrees
        "panda_joint7":  math.pi / 4,   #  45 degrees
    }
    robot_state.set_joint_group_positions("panda_arm", list(joint_values.values()))
    
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(robot_state=robot_state)
    
    plan_result = panda_arm.plan()
    if plan_result:
        print("[INFO] Custom joint plan successful! Executing...")
        robot = moveit.get_robot()
        robot.execute(plan_result.trajectory, controllers=[])
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Run it:**
```bash
# Terminal 2:
ros2 run panda_moveit2_tutorials basic_motion_planner
```

---

## Example 2: Cartesian Pose Planning

**File:** [`src/panda_moveit2_tutorials/panda_moveit2_tutorials/pose_goal_planner.py`](../src/panda_moveit2_tutorials/panda_moveit2_tutorials/pose_goal_planner.py)

```python
#!/usr/bin/env python3
"""
Lesson 03 - Example 2: Plan to a Cartesian end-effector pose.
"""

import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="panda_pose_planner")
    panda_arm = moveit.get_planning_component("panda_arm")
    
    # ─── Define target pose ─────────────────────────────────────────
    target_pose = PoseStamped()
    target_pose.header.frame_id = "panda_link0"
    
    # Position (x, y, z in meters from robot base)
    target_pose.pose.position.x = 0.307
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.590
    
    # Orientation (quaternion — this is pointing straight down)
    target_pose.pose.orientation.x = 0.924
    target_pose.pose.orientation.y = -0.383
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0
    
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(
        pose_stamped_msg=target_pose,
        pose_link="panda_link8"   # the end-effector link
    )
    
    plan_result = panda_arm.plan()
    
    if plan_result:
        print(f"[INFO] Plan found! Trajectory has "
              f"{len(plan_result.trajectory.joint_trajectory.points)} waypoints.")
        robot = moveit.get_robot()
        robot.execute(plan_result.trajectory, controllers=[])
    else:
        print("[WARN] No plan found.")
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## Example 3: Cartesian Waypoint Path

```python
#!/usr/bin/env python3
"""
Lesson 03 - Example 3: Follow a Cartesian waypoint path.
The robot traces a straight line in task space.
"""

import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="panda_cartesian_planner")
    panda_arm = moveit.get_planning_component("panda_arm")
    
    # ─── Define waypoints ────────────────────────────────────────────
    waypoints = []
    
    # Start position
    p1 = Pose()
    p1.position.x = 0.307
    p1.position.y = 0.0
    p1.position.z = 0.590
    p1.orientation.w = 1.0
    
    # Move 15cm forward
    p2 = Pose()
    p2.position.x = 0.457
    p2.position.y = 0.0
    p2.position.z = 0.590
    p2.orientation.w = 1.0
    
    # Move 15cm to the right
    p3 = Pose()
    p3.position.x = 0.457
    p3.position.y = -0.15
    p3.position.z = 0.590
    p3.orientation.w = 1.0
    
    waypoints = [p1, p2, p3]
    
    # ─── Compute Cartesian path ──────────────────────────────────────
    # eef_step: max distance between consecutive waypoints (meters)
    # jump_threshold: 0.0 disables jump detection
    (plan, fraction) = panda_arm.compute_cartesian_path(
        waypoints=waypoints,
        eef_step=0.01,       # 1cm resolution
        jump_threshold=0.0
    )
    
    print(f"[INFO] Cartesian path computed: {fraction*100:.1f}% achievable")
    
    if fraction > 0.9:   # At least 90% of path achievable
        robot = moveit.get_robot()
        robot.execute(plan, controllers=[])
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## Example 4: Adding Collision Objects

```python
#!/usr/bin/env python3
"""
Lesson 03 - Example 4: Add a box to the planning scene,
plan around it, then remove it.
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="scene_manager")
    panda_arm = moveit.get_planning_component("panda_arm")
    planning_scene_monitor = moveit.get_planning_scene_monitor()
    
    # ─── Add a box obstacle ──────────────────────────────────────────
    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "panda_link0"
        collision_object.id = "obstacle_box"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.1, 0.4, 0.4]
        
        box_pose = Pose()
        box_pose.position.x = 0.4
        box_pose.position.y = 0.0
        box_pose.position.z = 0.2
        box_pose.orientation.w = 1.0
        
        collision_object.primitives = [box]
        collision_object.primitive_poses = [box_pose]
        collision_object.operation = CollisionObject.ADD
        
        scene.apply_collision_object(collision_object)
        scene.current_state.update()
    
    print("[INFO] Obstacle box added to planning scene.")
    time.sleep(1.0)
    
    # ─── Plan around the obstacle ────────────────────────────────────
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="extended")
    plan_result = panda_arm.plan()
    
    if plan_result:
        print("[INFO] Found a collision-free path around the box!")
        robot = moveit.get_robot()
        robot.execute(plan_result.trajectory, controllers=[])
    else:
        print("[WARN] Could not find a collision-free path.")
    
    # ─── Remove the obstacle ─────────────────────────────────────────
    with planning_scene_monitor.read_write() as scene:
        scene.remove_world_object("obstacle_box")
        scene.current_state.update()
    
    print("[INFO] Obstacle box removed.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## Planner Configuration (Python)

```python
from moveit.planning import PlanRequestParameters

plan_params = PlanRequestParameters(moveit, "panda_arm")
plan_params.planner_id = "RRTConnectkConfigDefault"
plan_params.planning_time = 5.0        # max 5 seconds
plan_params.planning_attempts = 10     # retry up to 10 times
plan_params.max_velocity_scaling_factor = 0.5    # 50% of max speed
plan_params.max_acceleration_scaling_factor = 0.1 # 10% of max accel

plan_result = panda_arm.plan(single_plan_parameters=plan_params)
```

---

## Planning Time vs. Attempts — Understanding the Trade-off

```
Success Rate (%) vs Planning Time Budget
─────────────────────────────────────────────────────
1 second   │████████████████████████████░░░│ ~88%
2 seconds  │████████████████████████████████│ ~96%
5 seconds  │████████████████████████████████│ ~99%
10 seconds │████████████████████████████████│ ~99.9%

(Random goals, Panda arm, RRTConnect planner)
```

---

## Exercises

### 03-A: Move to a Named Configuration
Run `basic_motion_planner.py` and observe the robot moving to `ready`.

### 03-B: Custom Pose Goal
Modify `pose_goal_planner.py` to reach:
- position: `(0.4, 0.2, 0.4)`
- orientation: `(0.0, 0.0, 0.0, 1.0)` (neutral)

### 03-C: Draw a Square
Modify `cartesian_path_planner.py` to trace a 20cm × 20cm square in the XY plane at Z=0.5m.

---

## Next Steps

👉 Proceed to [Lesson 04 — C++ Motion Planning](04_motion_planning_cpp.md)
