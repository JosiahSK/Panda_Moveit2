#!/usr/bin/env python3
"""
Pick and place pipeline — complete 9-stage sequence with collision objects
and dynamic attachment/detachment to the robot gripper.
Lesson 06.
"""

import time
import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from panda_moveit2_tutorials.moveit_config_helper import (
    get_moveit_config_dict,
    ensure_valid_start_state,
    execute_trajectory,
    clean_exit,
)


class PandaPickPlace:
    TABLE_HEIGHT = 0.40
    OBJECT_RADIUS = 0.02
    OBJECT_HEIGHT = 0.10

    # Pick & place target positions (relative to panda_link0)
    # Using 0.40m reach with 0.15m lateral displacement for safe collision-free kinematics
    PICK_X, PICK_Y = 0.40, 0.15
    PLACE_X, PLACE_Y = 0.40, -0.15

    TABLE_Z = TABLE_HEIGHT / 2.0
    OBJECT_Z = TABLE_HEIGHT + (OBJECT_HEIGHT / 2.0) + 0.002  # 0.452m (2mm clearance above 0.40m table)
    APPROACH_OFFSET = 0.20                                   # 0.652m approach height (clearance above 0.502m can top)
    GRASP_Z = 0.582                                          # Link8 height for finger grasp without palm collision

    def __init__(self):
        print("[INFO] Initializing MoveItPy for pick_and_place...")
        self.moveit = MoveItPy(node_name="pick_and_place", config_dict=get_moveit_config_dict())
        self.arm = self.moveit.get_planning_component("panda_arm")
        self.gripper = self.moveit.get_planning_component("hand")
        self.psm = self.moveit.get_planning_scene_monitor()
        self._is_attached = False

    def setup_scene(self):
        """Add support surface and target cylinder object to the PlanningScene."""
        print("[SCENE] Setting up support surface and target object in planning scene...")
        # 1. Support table placed with 0.35m clearance from robot base
        table = CollisionObject()
        table.header.frame_id = "panda_link0"
        table.id = "support_table"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.40, 0.80, self.TABLE_HEIGHT]
        t_pose = PoseStamped()
        t_pose.header.frame_id = "panda_link0"
        t_pose.pose.position.x = 0.55
        t_pose.pose.position.y = 0.0
        t_pose.pose.position.z = self.TABLE_Z
        t_pose.pose.orientation.w = 1.0
        table.primitives.append(box)
        table.primitive_poses.append(t_pose.pose)
        table.operation = CollisionObject.ADD

        # 2. Target cylinder
        target_obj = CollisionObject()
        target_obj.header.frame_id = "panda_link0"
        target_obj.id = "target_can"
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [self.OBJECT_HEIGHT, self.OBJECT_RADIUS]
        c_pose = PoseStamped()
        c_pose.header.frame_id = "panda_link0"
        c_pose.pose.position.x = self.PICK_X
        c_pose.pose.position.y = self.PICK_Y
        c_pose.pose.position.z = self.OBJECT_Z
        c_pose.pose.orientation.w = 1.0
        target_obj.primitives.append(cyl)
        target_obj.primitive_poses.append(c_pose.pose)
        target_obj.operation = CollisionObject.ADD

        with self.psm.read_write() as scene:
            scene.apply_collision_object(table)
            scene.apply_collision_object(target_obj)
        time.sleep(0.5)
        print("[SCENE] Planning scene ready with table and target object.")

    def attach_object(self):
        """Attach target cylinder to panda_hand for kinematic collision awareness during transfer.

        Educational Note:
        In this simulation tutorial, attaching modifies MoveIt's Allowed Collision
        Matrix (ACM) via touch_links so that the gripper fingers can close around the object
        without FCL collision aborts. This is purely a kinematic planning model; it does NOT
        simulate physical grasping dynamics and does not prove physical grasp acquisition.
        """
        att = AttachedCollisionObject()
        att.link_name = "panda_hand"
        att.object.header.frame_id = "panda_hand"
        att.object.id = "target_can"
        att.touch_links = ["panda_leftfinger", "panda_rightfinger", "panda_hand"]
        att.object.operation = CollisionObject.ADD
        with self.psm.read_write() as scene:
            scene.process_attached_collision_object(att)
        self._is_attached = True
        print("[SCENE] Object 'target_can' attached to panda_hand.")

    def detach_object(self):
        """Detach object from gripper upon placement."""
        if not self._is_attached:
            return
        att = AttachedCollisionObject()
        att.link_name = "panda_hand"
        att.object.id = "target_can"
        att.object.operation = CollisionObject.REMOVE
        with self.psm.read_write() as scene:
            scene.process_attached_collision_object(att)
        self._is_attached = False
        print("[SCENE] Object 'target_can' detached from panda_hand.")

    def cleanup_scene(self):
        """Clean up scene collision objects and ensure detachment."""
        print("[SCENE] Cleaning up collision objects...")
        if self._is_attached:
            self.detach_object()

        table = CollisionObject()
        table.header.frame_id = "panda_link0"
        table.id = "support_table"
        table.operation = CollisionObject.REMOVE

        target_obj = CollisionObject()
        target_obj.header.frame_id = "panda_link0"
        target_obj.id = "target_can"
        target_obj.operation = CollisionObject.REMOVE

        with self.psm.read_write() as scene:
            scene.apply_collision_object(table)
            scene.apply_collision_object(target_obj)

    def _move_arm(self, x, y, z):
        target = PoseStamped()
        target.header.frame_id = "panda_link0"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        # Gripper pointing downward
        target.pose.orientation.x = 1.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")
        plan_res = self.arm.plan()
        if plan_res and plan_res.trajectory and len(plan_res.trajectory) > 0:
            execute_trajectory(self.moveit, plan_res.trajectory)
            time.sleep(0.3)
            return True
        else:
            print(f"[WARN] Failed planning arm motion to ({x:.2f}, {y:.2f}, {z:.2f})")
            return False

    def open_gripper(self):
        print("[INFO] Commanding gripper fingers to 'open'...")
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="open")
        p = self.gripper.plan()
        if p and p.trajectory:
            execute_trajectory(self.moveit, p.trajectory)
            time.sleep(0.3)
            return True
        print("[WARN] Failed planning gripper 'open' motion.")
        return False

    def close_gripper(self):
        print("[INFO] Commanding gripper fingers to 'close'...")
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="close")
        p = self.gripper.plan()
        if p and p.trajectory:
            execute_trajectory(self.moveit, p.trajectory)
            time.sleep(0.3)
            return True
        print("[WARN] Failed planning gripper 'close' motion.")
        return False

    def execute_pipeline(self):
        print("\n" + "=" * 70)
        print(" 🦾 STARTING COMPLETE 9-STAGE PICK & PLACE SEQUENCE")
        print(" Note: Simulated pick-and-place uses AttachedCollisionObject")
        print(" for kinematic collision geometry transfer.")
        print("=" * 70)

        # Stage 1: Home
        print("Stage 1: Moving arm to 'ready' home configuration...")
        ensure_valid_start_state(self.arm)
        self.arm.set_goal_state(configuration_name="ready")
        p = self.arm.plan()
        if p and p.trajectory:
            execute_trajectory(self.moveit, p.trajectory)
        else:
            print("[WARN] Failed planning arm motion to 'ready' pose.")
            return False

        # Stage 2: Approach Pick (Pre-grasp)
        print("Stage 2: Moving to pre-grasp approach pose above object...")
        if not self._move_arm(self.PICK_X, self.PICK_Y, self.OBJECT_Z + self.APPROACH_OFFSET):
            return False

        # Stage 3: Open gripper
        print("Stage 3: Opening gripper fingers...")
        if not self.open_gripper():
            return False

        # Stage 4: Descend to grasp
        print("Stage 4: Descending to grasp height...")
        if not self._move_arm(self.PICK_X, self.PICK_Y, self.GRASP_Z):
            return False

        # Stage 5: Attach & Grasp
        # Educational Note on Simulation vs. Real Hardware:
        # In MoveIt kinematic simulation, commanding the gripper to 'close' (0.0m) while
        # an unattached collision object sits between the fingers causes FCL collision
        # detection to fail because the fingers attempt to penetrate the solid object.
        # Calling attach_object() first adds touch_links to the Allowed Collision Matrix (ACM),
        # allowing collision-free trajectory planning.
        # IMPORTANT: Planning-scene attachment only updates MoveIt's geometric collision
        # model; it does NOT simulate physical grasping or prove the object is held.
        # On real hardware, the gripper would use force/current sensing (e.g., Franka Grasp
        # action) and attach_object() should only be called AFTER confirming contact feedback.
        print("Stage 5: Attaching object geometrically and closing gripper...")
        self.attach_object()
        if not self.close_gripper():
            return False

        # Stage 6: Lift object
        print("Stage 6: Lifting object vertically...")
        if not self._move_arm(self.PICK_X, self.PICK_Y, self.OBJECT_Z + self.APPROACH_OFFSET):
            return False

        # Stage 7: Transfer to place approach
        print("Stage 7: Transferring object to place position...")
        if not self._move_arm(self.PLACE_X, self.PLACE_Y, self.OBJECT_Z + self.APPROACH_OFFSET):
            return False

        # Stage 8: Lower to place surface
        print("Stage 8: Lowering object onto place surface...")
        if not self._move_arm(self.PLACE_X, self.PLACE_Y, self.GRASP_Z):
            return False

        # Stage 9: Release, Detach, and Retreat
        print("Stage 9: Releasing object and retreating...")
        if not self.open_gripper():
            return False
        self.detach_object()
        if not self._move_arm(self.PLACE_X, self.PLACE_Y, self.OBJECT_Z + self.APPROACH_OFFSET):
            return False

        # Return Home
        print("Returning arm to home 'ready' pose...")
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="ready")
        p = self.arm.plan()
        if p and p.trajectory:
            execute_trajectory(self.moveit, p.trajectory)
        else:
            print("[WARN] Failed planning return to 'ready' pose.")
            return False

        print("\n🎉 PICK AND PLACE PIPELINE SUCCESSFULLY COMPLETED!\n")
        return True


def main():
    rclpy.init()
    demo = PandaPickPlace()
    demo.setup_scene()
    success = demo.execute_pipeline()
    demo.cleanup_scene()
    clean_exit(0 if success else 1)


if __name__ == "__main__":
    main()
