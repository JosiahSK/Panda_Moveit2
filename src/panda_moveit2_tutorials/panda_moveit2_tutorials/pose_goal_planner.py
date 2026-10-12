#!/usr/bin/env python3
"""
Pose goal planner — plans to a Cartesian end-effector pose.
Lesson 03, Example 2.
"""

import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="panda_pose_planner")
    panda_arm = moveit.get_planning_component("panda_arm")
    robot = moveit.get_robot()

    target = PoseStamped()
    target.header.frame_id = "panda_link0"
    target.pose.position.x = 0.307
    target.pose.position.y = 0.0
    target.pose.position.z = 0.590
    target.pose.orientation.x = 0.924
    target.pose.orientation.y = -0.383
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 0.0

    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")

    result = panda_arm.plan()
    if result:
        n = len(result.trajectory.joint_trajectory.points)
        print(f"[INFO] Plan found: {n} waypoints. Executing...")
        robot.execute(result.trajectory, controllers=[])
    else:
        print("[WARN] No plan found for pose goal.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
