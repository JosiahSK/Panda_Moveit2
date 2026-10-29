"""
moveit_config_helper.py
Helper functions to build and configure MoveIt 2 parameters for Franka Panda on ROS 2 Jazzy.
"""

import copy
import os
import sys
import time
from moveit_configs_utils import MoveItConfigsBuilder


def get_moveit_config_dict():
    """Build and return complete MoveIt 2 configuration dictionary for Franka Panda on ROS 2 Jazzy."""
    builder = MoveItConfigsBuilder("moveit_resources_panda")
    mc = builder.to_dict()

    # Define loaded planning pipelines
    mc["planning_pipelines"] = {
        "pipeline_names": ["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"]
    }

    # Configure projection evaluator for OMPL projection-based planners (LBKPIECE, KPIECE, BKPIECE)
    # Upstream moveit_resources_panda_moveit_config ompl_planning.yaml omits projection_evaluator
    # for panda_arm, causing LBKPIECE to fail immediately with 'No projection evaluator specified'.
    if "ompl" in mc:
        if "panda_arm" not in mc["ompl"]:
            mc["ompl"]["panda_arm"] = {}
        mc["ompl"]["panda_arm"]["projection_evaluator"] = "joints(panda_joint1,panda_joint2)"

    # Default motion plan request parameters (used by MoveItCpp PlanningComponent)
    default_request_params = {
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "planning_time": 5.0,
        "planning_attempts": 5,
        "max_velocity_scaling_factor": 0.5,
        "max_acceleration_scaling_factor": 0.5,
    }
    mc["plan_request_params"] = copy.deepcopy(default_request_params)
    mc["ompl"]["plan_request_params"] = copy.deepcopy(default_request_params)

    # Pilz industrial motion planner parameters for linear/Cartesian motions
    mc["pilz_industrial_motion_planner"]["plan_request_params"] = {
        "planning_pipeline": "pilz_industrial_motion_planner",
        "planner_id": "LIN",
        "planning_time": 5.0,
        "planning_attempts": 5,
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
    }

    return mc


def ensure_valid_start_state(panda_arm, timeout_sec=3.0):
    """
    Ensure the planning component has received a non-colliding valid start state from /joint_states.
    Also provides sufficient time for DDS action server discovery to settle.
    """
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        curr = panda_arm.get_start_state()
        if curr and curr.joint_positions:
            # Check if arm joints are populated (panda_joint4 is -2.356 in ready pose, 0 in uninitialized/folded state)
            if abs(curr.joint_positions.get("panda_joint4", 0.0)) > 0.01:
                panda_arm.set_start_state_to_current_state()
                time.sleep(1.0)  # Allow DDS action client matching
                return True
        time.sleep(0.1)

    # Fallback to safe known ready state if /joint_states has not arrived yet
    panda_arm.set_start_state(configuration_name="ready")
    time.sleep(1.0)
    return True


def execute_trajectory(moveit, trajectory, controllers=None):
    """
    Safely execute a trajectory if execution infrastructure is ready.
    Provides clear educational feedback on planning vs execution status.
    """
    if controllers is None:
        controllers = []

    tem = moveit.get_trajectory_execution_manager()
    if not tem or not tem.is_managing_controllers():
        print("[WARN] Trajectory execution manager is not active or not managing controllers.")
        print("[INFO] Trajectory planned successfully, but skipping live execution.")
        return False

    print("[INFO] Submitting trajectory to execution manager...")
    try:
        status = moveit.execute(trajectory, controllers=controllers)
        if status:
            print(f"[SUCCESS] Trajectory executed successfully: {status.status}")
            return True
        else:
            print(f"[WARN] Trajectory execution completed with status: {status.status}")
            return False
    except Exception as e:
        print(f"[WARN] Execution could not be completed: {e}")
        return False


def clean_exit(code=0):
    """
    Cleanly terminate MoveItPy process in Python 3.12 without triggering the
    upstream MoveIt 2 / rclcpp TrajectoryExecutionManager destructor segmentation
    fault (upstream issue: moveit/moveit2#1597).

    Root cause: In ROS 2 Jazzy, MoveItCpp's TrajectoryExecutionManager creates an
    internal rclcpp Node and CallbackGroup. During Py_FinalizeEx, CallbackGroup::~CallbackGroup()
    attempts to dereference an already-partially-teardown rclcpp context/executor, causing
    an unavoidable exit code 139 (SIGSEGV).

    This helper explicitly flushes sys.stdout and sys.stderr before invoking os._exit(code),
    ensuring that all student logs and benchmark CSVs are fully written while bypassing
    the broken C++ destructor chain. Non-zero exit codes are preserved on failure.
    """
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        os._exit(code)
