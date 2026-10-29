#!/usr/bin/env python3
"""
Basic motion planner — plans to named states and joint values.
Lesson 03, Example 1.
"""

import math
import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from panda_moveit2_tutorials.moveit_config_helper import (
    get_moveit_config_dict,
    ensure_valid_start_state,
    execute_trajectory,
    clean_exit,
)


def main():
    rclpy.init()
    print("[INFO] Initializing MoveItPy for basic_motion_planner...")
    moveit = MoveItPy(node_name="basic_motion_planner", config_dict=get_moveit_config_dict())
    panda_arm = moveit.get_planning_component("panda_arm")

    print("[INFO] Connected to MoveIt 2. Planning to named state 'ready'...")

    # Plan to named state
    ensure_valid_start_state(panda_arm)
    panda_arm.set_goal_state(configuration_name="ready")
    plan_ready = panda_arm.plan()
    if plan_ready and plan_ready.trajectory and len(plan_ready.trajectory) > 0:
        print(f"[SUCCESS] Planning to 'ready' succeeded ({len(plan_ready.trajectory)} trajectory waypoints).")
        execute_trajectory(moveit, plan_ready.trajectory)
    else:
        print("[ERROR] Planning to 'ready' failed.")
        clean_exit(1)

    # Plan to custom joint values
    print("[INFO] Planning to custom joint configuration...")
    robot_model = moveit.get_robot_model()
    robot_state = RobotState(robot_model)

    joint_values = [
        0.0,
        -math.pi / 4.0,
        0.0,
        -math.pi / 2.0,
        0.0,
        math.pi / 3.0,
        math.pi / 4.0,
    ]
    robot_state.set_joint_group_positions("panda_arm", joint_values)

    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(robot_state=robot_state)
    plan_joint = panda_arm.plan()
    if plan_joint and plan_joint.trajectory and len(plan_joint.trajectory) > 0:
        print(f"[SUCCESS] Planning to custom joint configuration succeeded ({len(plan_joint.trajectory)} waypoints).")
        execute_trajectory(moveit, plan_joint.trajectory)
    else:
        print("[ERROR] Planning to custom joint configuration failed.")
        clean_exit(1)

    print("[INFO] basic_motion_planner execution completed successfully.")
    clean_exit(0)


if __name__ == "__main__":
    main()
