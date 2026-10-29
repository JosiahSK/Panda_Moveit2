#!/usr/bin/env python3
"""
Collision avoidance and scene obstacles demo.
Spawns table and wall obstacle in the planning scene, and plans a collision-free detour.
Lesson 03 & 04.
"""

import time
import rclpy
from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from panda_moveit2_tutorials.moveit_config_helper import (
    get_moveit_config_dict,
    ensure_valid_start_state,
    execute_trajectory,
    clean_exit,
)


def add_box_obstacle(moveit, object_id, size, position):
    """Publish a collision box to the planning scene."""
    psm = moveit.get_planning_scene_monitor()

    col_obj = CollisionObject()
    col_obj.header.frame_id = "panda_link0"
    col_obj.id = object_id

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = size  # [dx, dy, dz]
    col_obj.primitives.append(box)

    pose = PoseStamped()
    pose.header.frame_id = "panda_link0"
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.w = 1.0
    col_obj.primitive_poses.append(pose.pose)
    col_obj.operation = CollisionObject.ADD

    with psm.read_write() as scene:
        scene.apply_collision_object(col_obj)
    print(f"[SCENE] Added collision object: '{object_id}' size={size} at {position}")


def remove_obstacle(moveit, object_id):
    """Remove a collision object from the planning scene."""
    psm = moveit.get_planning_scene_monitor()
    col_obj = CollisionObject()
    col_obj.header.frame_id = "panda_link0"
    col_obj.id = object_id
    col_obj.operation = CollisionObject.REMOVE

    with psm.read_write() as scene:
        scene.apply_collision_object(col_obj)
    print(f"[SCENE] Removed collision object: '{object_id}'")


def main():
    rclpy.init()
    print("[INFO] Initializing MoveItPy for collision_avoidance_demo...")
    moveit = MoveItPy(node_name="collision_avoidance_demo", config_dict=get_moveit_config_dict())
    panda_arm = moveit.get_planning_component("panda_arm")

    print("[INFO] Step 1: Setting baseline 'ready' pose...")
    ensure_valid_start_state(panda_arm)
    panda_arm.set_goal_state(configuration_name="ready")
    plan_ready = panda_arm.plan()
    if plan_ready:
        execute_trajectory(moveit, plan_ready.trajectory)

    time.sleep(0.5)

    # 1. Add table surface
    add_box_obstacle(moveit, "table", [0.8, 1.2, 0.4], [0.5, 0.0, 0.2])

    # 2. Add barrier obstacle between start and goal
    add_box_obstacle(moveit, "wall_barrier", [0.10, 0.20, 0.20], [0.40, 0.0, 0.30])
    time.sleep(0.5)

    # 3. Plan to goal behind the obstacle
    target = PoseStamped()
    target.header.frame_id = "panda_link0"
    target.pose.position.x = 0.45
    target.pose.position.y = 0.0
    target.pose.position.z = 0.60
    target.pose.orientation.x = 0.924
    target.pose.orientation.y = -0.383
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 0.0

    print("[INFO] Step 2: Planning collision-free trajectory over and around obstacles...")
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")

    result = panda_arm.plan()
    if result:
        print(f"[SUCCESS] Found collision-free path around obstacles ({len(result.trajectory)} waypoints)!")
        execute_trajectory(moveit, result.trajectory)
        time.sleep(1.0)
    else:
        print("[ERROR] Could not find collision-free trajectory.")

    # 4. Clean up scene
    print("[INFO] Step 3: Cleaning up planning scene...")
    remove_obstacle(moveit, "wall_barrier")
    remove_obstacle(moveit, "table")

    # Return home
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    plan_home = panda_arm.plan()
    if plan_home:
        execute_trajectory(moveit, plan_home.trajectory)

    print("[INFO] collision_avoidance_demo execution completed.")
    clean_exit(0 if result else 1)


if __name__ == "__main__":
    main()
