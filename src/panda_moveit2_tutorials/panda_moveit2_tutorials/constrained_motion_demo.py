#!/usr/bin/env python3
"""
Path and orientation constraints planner node.
Enforces that the gripper remains strictly pointing downwards (carrying water cup demo).
Lesson 05.
"""

import time
import rclpy
from moveit.planning import MoveItPy
from moveit_msgs.msg import Constraints, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from panda_moveit2_tutorials.moveit_config_helper import (
    get_moveit_config_dict,
    ensure_valid_start_state,
    execute_trajectory,
    clean_exit,
)


def main():
    rclpy.init()
    print("[INFO] Initializing MoveItPy for constrained_motion_demo...")
    moveit = MoveItPy(node_name="constrained_motion_demo", config_dict=get_moveit_config_dict())
    panda_arm = moveit.get_planning_component("panda_arm")

    print("[INFO] Step 1: Moving to baseline 'ready' pose...")
    ensure_valid_start_state(panda_arm)
    panda_arm.set_goal_state(configuration_name="ready")
    plan_ready = panda_arm.plan()
    if plan_ready:
        execute_trajectory(moveit, plan_ready.trajectory)

    time.sleep(0.5)

    # Define Upright Orientation Constraint
    # Gripper tool axis must remain within +/- 0.35 rad of vertical during transfer
    constraints = Constraints()
    oc = OrientationConstraint()
    oc.header.frame_id = "panda_link0"
    oc.link_name = "panda_link8"
    oc.orientation.x = 0.924
    oc.orientation.y = -0.383
    oc.orientation.z = 0.0
    oc.orientation.w = 0.0
    oc.absolute_x_axis_tolerance = 0.35
    oc.absolute_y_axis_tolerance = 0.35
    oc.absolute_z_axis_tolerance = 3.14  # Unconstrained rotation about end-effector z-axis
    oc.weight = 1.0
    constraints.orientation_constraints.append(oc)

    # Goal Pose across the workspace
    target = PoseStamped()
    target.header.frame_id = "panda_link0"
    target.pose.position.x = 0.35
    target.pose.position.y = -0.20
    target.pose.position.z = 0.45
    target.pose.orientation.x = 0.924
    target.pose.orientation.y = -0.383
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 0.0

    print("[INFO] Step 2: Planning motion with strict upright end-effector orientation constraint...")
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")
    panda_arm.set_path_constraints(constraints)

    result = panda_arm.plan()
    if result:
        print(f"[SUCCESS] Orientation-constrained path found ({len(result.trajectory)} waypoints)!")
        execute_trajectory(moveit, result.trajectory)
        time.sleep(1.0)
    else:
        print("[WARN] Failed to find constrained trajectory with current tolerances.")

    # Step 3: Clear constraints using empty Constraints message and return home
    print("[INFO] Step 3: Clearing path constraints and returning home...")
    panda_arm.set_path_constraints(Constraints())
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    plan_home = panda_arm.plan()
    if plan_home:
        execute_trajectory(moveit, plan_home.trajectory)

    print("[INFO] constrained_motion_demo execution completed.")
    clean_exit(0 if result else 1)


if __name__ == "__main__":
    main()
