#!/usr/bin/env python3
"""
Pick and place pipeline — complete 9-stage sequence with collision objects
and dynamic attachment/detachment to the robot gripper.
Lesson 06 & Task 6.
"""

import rclpy
import time
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive


class PandaPickPlace:
    TABLE_HEIGHT = 0.40
    OBJECT_RADIUS = 0.025
    OBJECT_HEIGHT = 0.10
    
    # Coordinates relative to panda_link0
    PICK_X, PICK_Y = 0.45, 0.15
    PLACE_X, PLACE_Y = 0.45, -0.20
    
    TABLE_Z = TABLE_HEIGHT / 2.0
    OBJECT_Z = TABLE_HEIGHT + (OBJECT_HEIGHT / 2.0)
    APPROACH_OFFSET = 0.15

    def __init__(self):
        self.moveit = MoveItPy(node_name="pick_and_place_node")
        self.arm = self.moveit.get_planning_component("panda_arm")
        self.gripper = self.moveit.get_planning_component("hand")
        self.robot = self.moveit.get_robot()
        self.psm = self.moveit.get_planning_scene_monitor()

    def setup_scene(self):
        """Add support surface and target cylinder object to the PlanningScene."""
        print("[SCENE] Setting up support surface and target object...")
        # 1. Support table
        table = CollisionObject()
        table.header.frame_id = "panda_link0"
        table.id = "support_table"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.8, 1.2, self.TABLE_HEIGHT]
        t_pose = PoseStamped()
        t_pose.header.frame_id = "panda_link0"
        t_pose.pose.position.x = 0.50
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
            scene.current_state.update()
            scene.process_collision_object_msg(table)
            scene.process_collision_object_msg(target_obj)
        time.sleep(1.0)
        print("[SCENE] Planning scene ready with table and object.")

    def attach_object(self):
        """Attach the target cylinder to panda_hand to maintain collision awareness during transfer."""
        att = AttachedCollisionObject()
        att.link_name = "panda_hand"
        att.object.header.frame_id = "panda_hand"
        att.object.id = "target_can"
        att.touch_links = ["panda_leftfinger", "panda_rightfinger", "panda_hand"]
        att.object.operation = CollisionObject.ADD
        with self.psm.read_write() as scene:
            scene.process_attached_collision_object_msg(att)
        print("[SCENE] Object attached to panda_hand.")

    def detach_object(self):
        """Detach object from gripper upon placement."""
        att = AttachedCollisionObject()
        att.link_name = "panda_hand"
        att.object.id = "target_can"
        att.object.operation = CollisionObject.REMOVE
        with self.psm.read_write() as scene:
            scene.process_attached_collision_object_msg(att)
        print("[SCENE] Object detached from panda_hand.")

    def _move_arm(self, x, y, z):
        target = PoseStamped()
        target.header.frame_id = "panda_link0"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        # Gripper pointing downward
        target.pose.orientation.x = 0.924
        target.pose.orientation.y = -0.383
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")
        plan_res = self.arm.plan()
        if plan_res:
            self.robot.execute(plan_res.trajectory, controllers=[])
            time.sleep(0.5)
            return True
        return False

    def open_gripper(self):
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="open")
        p = self.gripper.plan()
        if p:
            self.robot.execute(p.trajectory, controllers=[])
            time.sleep(0.5)

    def close_gripper(self):
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="close")
        p = self.gripper.plan()
        if p:
            self.robot.execute(p.trajectory, controllers=[])
            time.sleep(0.5)

    def execute_pipeline(self):
        print("\n🚀 STARTING COMPLETE 9-STAGE PICK & PLACE SEQUENCE")
        
        # Stage 1: Home
        print("Stage 1: Moving to 'ready' home configuration...")
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="ready")
        p = self.arm.plan()
        if p:
            self.robot.execute(p.trajectory, controllers=[])

        # Stage 2: Approach Pick (Pre-grasp)
        print("Stage 2: Moving to pre-grasp approach pose above object...")
        if not self._move_arm(self.PICK_X, self.PICK_Y, self.OBJECT_Z + self.APPROACH_OFFSET):
            return False

        # Stage 3: Open gripper
        print("Stage 3: Opening fingers...")
        self.open_gripper()

        # Stage 4: Descend to grasp
        print("Stage 4: Descending to grasp height...")
        if not self._move_arm(self.PICK_X, self.PICK_Y, self.OBJECT_Z + 0.03):
            return False

        # Stage 5: Grasp & Attach
        print("Stage 5: Closing gripper and attaching object...")
        self.close_gripper()
        self.attach_object()

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
        if not self._move_arm(self.PLACE_X, self.PLACE_Y, self.OBJECT_Z + 0.03):
            return False

        # Stage 9: Release, Detach, and Retreat
        print("Stage 9: Releasing object and retreating...")
        self.open_gripper()
        self.detach_object()
        self._move_arm(self.PLACE_X, self.PLACE_Y, self.OBJECT_Z + self.APPROACH_OFFSET)

        # Return Home
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="ready")
        p = self.arm.plan()
        if p:
            self.robot.execute(p.trajectory, controllers=[])

        print("🎉 PICK AND PLACE PIPELINE SUCCESSFULLY COMPLETED!\n")
        return True


def main():
    rclpy.init()
    demo = PandaPickPlace()
    demo.setup_scene()
    demo.execute_pipeline()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
