#!/usr/bin/env python3
"""
Basic motion planner — plans to named states and joint values.
Lesson 03, Example 1.
"""

import rclpy
import math
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="panda_motion_planner")
    panda_arm = moveit.get_planning_component("panda_arm")
    robot = moveit.get_robot()

    print("[INFO] Connected to MoveIt 2. Planning to 'ready'...")

    # Plan to named state
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name="ready")
    result = panda_arm.plan()
    if result:
        robot.execute(result.trajectory, controllers=[])
        print("[INFO] Reached 'ready' state.")
    else:
        print("[WARN] Planning to 'ready' failed.")

    # Plan to custom joint values
    robot_model = moveit.get_robot_model()
    robot_state = RobotState(robot_model)

    joint_values = [0.0, -math.pi / 4, 0.0, -math.pi / 2,
                    0.0, math.pi / 3, math.pi / 4]
    robot_state.set_joint_group_positions("panda_arm", joint_values)

    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(robot_state=robot_state)
    result = panda_arm.plan()
    if result:
        robot.execute(result.trajectory, controllers=[])
        print("[INFO] Reached custom joint configuration.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
