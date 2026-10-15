#!/usr/bin/env python3
"""
Path and orientation constraints planner node.
Enforces that the gripper remains strictly pointing downwards (carrying water cup demo).
"""

import rclpy
import math
import time
from moveit.planning import MoveItPy
from moveit_msgs.msg import Constraints, OrientationConstraint
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="constraints_demo_node")
    panda_arm = moveit.get_planning_component("panda_arm")
    robot = moveit.get_robot()

    print("[INFO] Moving to initial pose...")
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    p = panda_arm.plan()
    if p:
        robot.execute(p.trajectory, controllers=[])

    time.sleep(1.0)

    # Define Upright Orientation Constraint
    # Gripper must remain within +/- 0.25 radians of vertical during the entire movement
    constraints = Constraints()
    oc = OrientationConstraint()
    oc.header.frame_id = "panda_link0"
    oc.link_name = "panda_link8"
    oc.orientation.x = 0.924
    oc.orientation.y = -0.383
    oc.orientation.z = 0.0
    oc.orientation.w = 0.0
    oc.absolute_x_axis_tolerance = 0.25
    oc.absolute_y_axis_tolerance = 0.25
    oc.absolute_z_axis_tolerance = 3.14  # Can spin around tool axis
    oc.weight = 1.0
    constraints.orientation_constraints.append(oc)

    # Goal Pose across the workspace
    target = PoseStamped()
    target.header.frame_id = "panda_link0"
    target.pose.position.x = 0.35
    target.pose.position.y = -0.30
    target.pose.position.z = 0.40
    target.pose.orientation.x = 0.924
    target.pose.orientation.y = -0.383
    target.pose.orientation.w = 0.0

    print("[INFO] Planning motion with strict EEF upright orientation constraint...")
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")
    panda_arm.set_path_constraints(constraints)

    result = panda_arm.plan()
    if result:
        print("[SUCCESS] Constraint-compliant path found! Executing upright sweep in RViz...")
        robot.execute(result.trajectory, controllers=[])
        time.sleep(2.0)
    else:
        print("[WARN] Failed to find constrained trajectory. Try increasing planning time.")

    # Clear constraints and return
    panda_arm.clear_path_constraints()
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    p = panda_arm.plan()
    if p:
        robot.execute(p.trajectory, controllers=[])

    rclpy.shutdown()


if __name__ == "__main__":
    main()
