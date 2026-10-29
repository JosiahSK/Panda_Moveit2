#!/usr/bin/env python3
"""
Pose goal planner — plans to a Cartesian end-effector pose.
Lesson 03, Example 2.
"""

import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from panda_moveit2_tutorials.moveit_config_helper import (
    get_moveit_config_dict,
    ensure_valid_start_state,
    execute_trajectory,
    clean_exit,
)


def main():
    rclpy.init()
    print("[INFO] Initializing MoveItPy for pose_goal_planner...")
    moveit = MoveItPy(node_name="pose_goal_planner", config_dict=get_moveit_config_dict())
    panda_arm = moveit.get_planning_component("panda_arm")

    target = PoseStamped()
    target.header.frame_id = "panda_link0"
    target.pose.position.x = 0.307
    target.pose.position.y = 0.0
    target.pose.position.z = 0.590
    target.pose.orientation.x = 0.924
    target.pose.orientation.y = -0.383
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 0.0

    print(f"[INFO] Planning motion to target pose: ({target.pose.position.x}, {target.pose.position.y}, {target.pose.position.z})...")
    ensure_valid_start_state(panda_arm)
    panda_arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")

    result = panda_arm.plan()
    if result and result.trajectory and len(result.trajectory) > 0:
        n = len(result.trajectory)
        print(f"[SUCCESS] Plan found: {n} trajectory waypoints.")
        execute_trajectory(moveit, result.trajectory)
        print("[INFO] pose_goal_planner execution completed successfully.")
        clean_exit(0)
    else:
        print("[ERROR] No valid trajectory found for target pose.")
        clean_exit(1)


if __name__ == "__main__":
    main()
