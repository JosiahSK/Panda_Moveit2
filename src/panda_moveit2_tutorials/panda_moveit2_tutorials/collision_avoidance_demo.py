#!/usr/bin/env python3
"""
Collision avoidance and scene obstacles demo.
Spawns table, wall obstacle, and plans collision-free detour.
"""

import rclpy
import time
from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


def add_box_obstacle(moveit, object_id, size, position):
    """Publish a collision box to the planning scene."""
    planning_scene_monitor = moveit.get_planning_scene_monitor()
    
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

    with planning_scene_monitor.read_write() as scene:
        scene.current_state.update()
        scene.process_collision_object_msg(col_obj)
    print(f"[SCENE] Added collision object: '{object_id}' at {position}")


def remove_obstacle(moveit, object_id):
    """Remove a collision object from the planning scene."""
    planning_scene_monitor = moveit.get_planning_scene_monitor()
    col_obj = CollisionObject()
    col_obj.header.frame_id = "panda_link0"
    col_obj.id = object_id
    col_obj.operation = CollisionObject.REMOVE

    with planning_scene_monitor.read_write() as scene:
        scene.current_state.update()
        scene.process_collision_object_msg(col_obj)
    print(f"[SCENE] Removed collision object: '{object_id}'")


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="collision_avoidance_demo")
    panda_arm = moveit.get_planning_component("panda_arm")
    robot = moveit.get_robot()

    print("[INFO] Setting initial 'ready' pose...")
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    p = panda_arm.plan()
    if p:
        robot.execute(p.trajectory, controllers=[])

    time.sleep(1.0)

    # 1. Add table surface
    add_box_obstacle(moveit, "table", [0.8, 1.2, 0.4], [0.5, 0.0, 0.2])

    # 2. Add tall wall obstacle directly between start and target
    add_box_obstacle(moveit, "wall_barrier", [0.08, 0.40, 0.50], [0.45, 0.0, 0.65])
    time.sleep(1.0)

    # 3. Plan to goal behind the wall
    target = PoseStamped()
    target.header.frame_id = "panda_link0"
    target.pose.position.x = 0.60
    target.pose.position.y = 0.15
    target.pose.position.z = 0.55
    target.pose.orientation.x = 0.924
    target.pose.orientation.y = -0.383
    target.pose.orientation.w = 0.0

    print("[INFO] Planning trajectory over and around the wall obstacle...")
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")

    result = panda_arm.plan()
    if result:
        print("[SUCCESS] Found collision-free path around barrier! Executing in RViz...")
        robot.execute(result.trajectory, controllers=[])
        time.sleep(2.0)
    else:
        print("[ERROR] Could not find collision-free trajectory.")

    # 4. Clean up scene
    print("[INFO] Cleaning up planning scene...")
    remove_obstacle(moveit, "wall_barrier")
    remove_obstacle(moveit, "table")

    # Return home
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    p = panda_arm.plan()
    if p:
        robot.execute(p.trajectory, controllers=[])

    rclpy.shutdown()


if __name__ == "__main__":
    main()
