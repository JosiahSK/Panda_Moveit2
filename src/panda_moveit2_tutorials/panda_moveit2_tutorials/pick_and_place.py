#!/usr/bin/env python3
"""
Pick and place pipeline — full 9-stage pick and place.
Lesson 06.
"""

import rclpy
import time
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


class PandaPickPlace:
    OBJECT_X, OBJECT_Y, OBJECT_Z = 0.40, 0.10, 0.06
    PLACE_X,  PLACE_Y,  PLACE_Z  = 0.40, -0.20, 0.06
    APPROACH = 0.12

    def __init__(self):
        self.moveit = MoveItPy(node_name="pick_and_place_node")
        self.arm = self.moveit.get_planning_component("panda_arm")
        self.gripper = self.moveit.get_planning_component("hand")
        self.robot = self.moveit.get_robot()

    def _move_to_pose(self, x, y, z, qx=0.924, qy=-0.383, qz=0.0, qw=0.0):
        t = PoseStamped()
        t.header.frame_id = "panda_link0"
        t.pose.position.x = x
        t.pose.position.y = y
        t.pose.position.z = z
        t.pose.orientation.x = qx
        t.pose.orientation.y = qy
        t.pose.orientation.z = qz
        t.pose.orientation.w = qw
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=t, pose_link="panda_link8")
        result = self.arm.plan()
        if result:
            self.robot.execute(result.trajectory, controllers=[])
            time.sleep(0.5)
            return True
        return False

    def open_gripper(self):
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="open")
        r = self.gripper.plan()
        if r:
            self.robot.execute(r.trajectory, controllers=[])
            time.sleep(0.8)

    def close_gripper(self):
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="close")
        r = self.gripper.plan()
        if r:
            self.robot.execute(r.trajectory, controllers=[])
            time.sleep(0.8)

    def pick(self):
        print("=== PICK ===")
        if not self._move_to_pose(self.OBJECT_X, self.OBJECT_Y, self.OBJECT_Z + self.APPROACH):
            return False
        self.open_gripper()
        if not self._move_to_pose(self.OBJECT_X, self.OBJECT_Y, self.OBJECT_Z):
            return False
        self.close_gripper()
        return self._move_to_pose(self.OBJECT_X, self.OBJECT_Y, self.OBJECT_Z + self.APPROACH + 0.05)

    def place(self):
        print("=== PLACE ===")
        if not self._move_to_pose(self.PLACE_X, self.PLACE_Y, self.PLACE_Z + self.APPROACH):
            return False
        if not self._move_to_pose(self.PLACE_X, self.PLACE_Y, self.PLACE_Z):
            return False
        self.open_gripper()
        return self._move_to_pose(self.PLACE_X, self.PLACE_Y, self.PLACE_Z + self.APPROACH)

    def home(self):
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="ready")
        r = self.arm.plan()
        if r:
            self.robot.execute(r.trajectory, controllers=[])


def main():
    rclpy.init()
    robot = PandaPickPlace()
    robot.home()
    time.sleep(1.0)
    if robot.pick() and robot.place():
        print("Pick and Place COMPLETE!")
    robot.home()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
