#!/usr/bin/env python3
"""
Cartesian Waypoint & Inspection Path Planner.
Plans straight-line Cartesian segments and a closed inspection square in 3D workspace.
Lesson 03 & 04.
"""

import time
import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import PoseStamped
from panda_moveit2_tutorials.moveit_config_helper import (
    get_moveit_config_dict,
    ensure_valid_start_state,
    execute_trajectory,
    clean_exit,
)


def main():
    rclpy.init()
    print("[INFO] Initializing MoveItPy for cartesian_path_planner...")
    moveit = MoveItPy(node_name="cartesian_path_planner", config_dict=get_moveit_config_dict())
    panda_arm = moveit.get_planning_component("panda_arm")

    # Step 1: Move to named 'ready' state to establish known baseline
    print("[INFO] Step 1: Moving to baseline 'ready' position...")
    ensure_valid_start_state(panda_arm)
    panda_arm.set_goal_state(configuration_name="ready")
    plan_ready = panda_arm.plan()
    if plan_ready:
        print(f"[SUCCESS] Reached baseline 'ready' state ({len(plan_ready.trajectory)} waypoints).")
        execute_trajectory(moveit, plan_ready.trajectory)
        time.sleep(0.5)

    # Step 2: Cartesian square / box inspection trajectory
    print("[INFO] Step 2: Planning Cartesian square inspection trajectory using Pilz LIN planner...")
    start_x, start_y, start_z = 0.35, -0.10, 0.45
    box_size = 0.15

    corners = [
        (start_x, start_y, start_z, "Bottom-Left"),
        (start_x, start_y + box_size, start_z, "Bottom-Right"),
        (start_x, start_y + box_size, start_z + box_size, "Top-Right"),
        (start_x, start_y, start_z + box_size, "Top-Left"),
        (start_x, start_y, start_z, "Return to Bottom-Left"),
    ]

    # Setup Pilz LIN request parameters
    lin_params = PlanRequestParameters(moveit, "pilz_industrial_motion_planner")
    lin_params.planning_pipeline = "pilz_industrial_motion_planner"
    lin_params.planner_id = "LIN"
    lin_params.planning_time = 3.0
    lin_params.max_velocity_scaling_factor = 0.1
    lin_params.max_acceleration_scaling_factor = 0.1

    successful_segments = 0
    total_segments = len(corners)

    for x, y, z, desc in corners:
        target = PoseStamped()
        target.header.frame_id = "panda_link0"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        # Keep gripper pointing downward
        target.pose.orientation.x = 0.924
        target.pose.orientation.y = -0.383
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0

        panda_arm.set_start_state_to_current_state()
        panda_arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")

        # Plan with Pilz LIN (linear Cartesian motion)
        result = panda_arm.plan(single_plan_parameters=lin_params)

        # Fallback to OMPL if linear Cartesian path encounters kinematic limits
        if not result:
            print(f"[INFO] Pilz LIN unavailable for {desc}; attempting OMPL plan...")
            result = panda_arm.plan()

        if result:
            successful_segments += 1
            print(f"[SUCCESS] Waypoint {desc} planned: {len(result.trajectory)} points (X={x:.2f}, Y={y:.2f}, Z={z:.2f})")
            execute_trajectory(moveit, result.trajectory)
            time.sleep(0.3)
        else:
            print(f"[WARN] Failed planning to waypoint: {desc}")

    coverage = (successful_segments / total_segments) * 100.0
    print(f"[INFO] Cartesian inspection completed: {successful_segments}/{total_segments} ({coverage:.1f}% path coverage).")

    # Step 3: Return to home
    print("[INFO] Step 3: Returning arm to 'ready' state...")
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    plan_home = panda_arm.plan()
    if plan_home:
        execute_trajectory(moveit, plan_home.trajectory)

    print("[INFO] cartesian_path_planner execution completed.")
    clean_exit(0 if successful_segments > 0 else 1)


if __name__ == "__main__":
    main()
