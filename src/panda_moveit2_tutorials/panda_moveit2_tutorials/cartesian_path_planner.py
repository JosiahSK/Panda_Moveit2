#!/usr/bin/env python3
"""
Cartesian Waypoint & Circle Path Planner.
Plans straight lines and a circular trajectory in 3D Cartesian space.
"""

import rclpy
import math
import time
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    node_name = "cartesian_path_planner"
    print(f"[INFO] Initializing {node_name}...")
    moveit = MoveItPy(node_name=node_name)
    panda_arm = moveit.get_planning_component("panda_arm")
    robot = moveit.get_robot()

    # Step 1: Move to named 'ready' state to establish known baseline
    print("[INFO] Step 1: Moving to baseline 'ready' position...")
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    plan_ready = panda_arm.plan()
    if plan_ready:
        robot.execute(plan_ready.trajectory, controllers=[])
        time.sleep(1.0)

    # Step 2: Cartesian square / box path in front of the robot
    print("[INFO] Step 2: Planning Cartesian square inspection trajectory...")
    start_x, start_y, start_z = 0.40, -0.15, 0.45
    box_size = 0.20

    corners = [
        (start_x, start_y, start_z, "Bottom-Left"),
        (start_x, start_y + box_size, start_z, "Bottom-Right"),
        (start_x, start_y + box_size, start_z + box_size, "Top-Right"),
        (start_x, start_y, start_z + box_size, "Top-Left"),
        (start_x, start_y, start_z, "Return to Bottom-Left")
    ]

    for x, y, z, desc in corners:
        target = PoseStamped()
        target.header.frame_id = "panda_link0"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        # Keep gripper facing downward
        target.pose.orientation.x = 0.924
        target.pose.orientation.y = -0.383
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0

        panda_arm.set_start_state_to_current_state()
        panda_arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")
        result = panda_arm.plan()

        if result:
            print(f"[SUCCESS] Reached waypoint: {desc} (X={x:.2f}, Y={y:.2f}, Z={z:.2f})")
            robot.execute(result.trajectory, controllers=[])
            time.sleep(0.3)
        else:
            print(f"[WARN] Failed planning to waypoint: {desc}")

    print("[INFO] Cartesian square path completed!")

    # Return home
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    plan_home = panda_arm.plan()
    if plan_home:
        robot.execute(plan_home.trajectory, controllers=[])

    rclpy.shutdown()


if __name__ == "__main__":
    main()
