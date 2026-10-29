#!/usr/bin/env python3
"""
Planner benchmarking script — rigorously compares solve times, success rates,
and trajectory metrics across OMPL planners (RRTConnect, PRM, LBKPIECE, RRTstar).
Exports results to CSV and prints summary metrics.
"""

import csv
import math
import os
import time

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from panda_moveit2_tutorials.moveit_config_helper import clean_exit, get_moveit_config_dict
import rclpy


def calculate_trajectory_length(trajectory):
    """Calculate joint-space Euclidean path length in radians."""
    if not trajectory:
        return 0.0
    try:
        msg = trajectory.get_robot_trajectory_msg()
        pts = msg.joint_trajectory.points
        if not pts:
            return 0.0
        total_length = 0.0
        for i in range(1, len(pts)):
            p_prev = pts[i - 1].positions
            p_curr = pts[i].positions
            dist = math.sqrt(sum((c - p) ** 2 for p, c in zip(p_prev, p_curr)))
            total_length += dist
        return total_length
    except Exception:
        return 0.0


def benchmark_planners():
    rclpy.init()
    node_name = "planner_benchmark"
    config_dict = get_moveit_config_dict()
    moveit = MoveItPy(node_name=node_name, config_dict=config_dict)
    panda_arm = moveit.get_planning_component("panda_arm")

    # Define standard target pose
    target = PoseStamped()
    target.header.frame_id = "panda_link0"
    target.pose.position.x = 0.40
    target.pose.position.y = 0.20
    target.pose.position.z = 0.50
    target.pose.orientation.w = 0.0
    target.pose.orientation.x = 1.0
    target.pose.orientation.y = 0.0
    target.pose.orientation.z = 0.0

    planners_to_test = [
        "RRTConnectkConfigDefault",
        "PRMkConfigDefault",
        "LBKPIECEkConfigDefault",
        "RRTstarkConfigDefault"
    ]
    trials_per_planner = 5
    planning_time_budget = 5.0

    print("\n" + "=" * 80)
    print(" 📊 RIGOROUS MOVEIT 2 OMPL PLANNER BENCHMARK (PANDA 7-DOF)")
    print(f" Trials per planner: {trials_per_planner} | Planning Time Budget: {planning_time_budget}s")
    print(" Note on RRT*: RRT* is an asymptotically optimal anytime planner. Unlike")
    print(" feasibility planners (RRTConnect, PRM, LBKPIECE) which return immediately upon")
    print(f" finding a path, RRT* utilizes its entire time budget ({planning_time_budget}s) to continually")
    print(" rewire and optimize path cost.")
    print("=" * 80)

    records = []

    for planner_id in planners_to_test:
        print(f"\n[BENCHMARK] Testing planner: {planner_id}")
        
        try:
            params = PlanRequestParameters(moveit, "ompl")
            params.planner_id = planner_id
            params.planning_time = planning_time_budget
        except Exception as e:
            print(f"  [ERROR] Failed to instantiate PlanRequestParameters for '{planner_id}': {e}")
            continue

        for trial in range(1, trials_per_planner + 1):
            panda_arm.set_start_state(configuration_name="ready")
            panda_arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")

            t_start = time.perf_counter()
            plan_res = panda_arm.plan(single_plan_parameters=params)
            t_end = time.perf_counter()

            latency_ms = (t_end - t_start) * 1000.0

            if plan_res and plan_res.trajectory and len(plan_res.trajectory) > 0:
                success = True
                pts_count = len(plan_res.trajectory)
                traj_len = calculate_trajectory_length(plan_res.trajectory)
                duration = plan_res.trajectory.duration
                print(f"  Trial {trial}: SUCCESS | Latency: {latency_ms:.2f} ms | Waypoints: {pts_count} | Duration: {duration:.2f}s | Path Length: {traj_len:.3f} rad")
            else:
                success = False
                pts_count = 0
                traj_len = 0.0
                duration = 0.0
                print(f"  Trial {trial}: FAILED | Latency: {latency_ms:.2f} ms")

            records.append({
                "planner": planner_id,
                "trial": trial,
                "success": success,
                "latency_ms": round(latency_ms, 2),
                "waypoints": pts_count,
                "duration_s": round(duration, 2),
                "path_length_rad": round(traj_len, 3)
            })

    # Export records to CSV
    csv_file = "/tmp/benchmark_results.csv"
    with open(csv_file, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["planner", "trial", "success", "latency_ms", "waypoints", "duration_s", "path_length_rad"])
        writer.writeheader()
        writer.writerows(records)

    print(f"\n[INFO] Benchmark data successfully exported to: {os.path.abspath(csv_file)}")

    # Print summary table
    print("\n" + "=" * 80)
    print(f"{'Planner ID':<26} | {'Success Rate':<13} | {'Avg Latency (ms)':<17} | {'Avg Path (rad)':<15}")
    print("-" * 80)
    for p_name in planners_to_test:
        p_records = [r for r in records if r["planner"] == p_name]
        successes = sum(1 for r in p_records if r["success"])
        rate = (successes / len(p_records)) * 100.0 if p_records else 0.0
        successful_lats = [r["latency_ms"] for r in p_records if r["success"]]
        avg_lat = sum(successful_lats) / len(successful_lats) if successful_lats else 0.0
        successful_lens = [r["path_length_rad"] for r in p_records if r["success"]]
        avg_len = sum(successful_lens) / len(successful_lens) if successful_lens else 0.0

        print(f"{p_name:<26} | {rate:<12.1f}% | {avg_lat:<17.2f} | {avg_len:<15.3f}")
    print("=" * 80)
    print(" Note: The observed success rate reflects a sample of 5 trials per planner")
    print(" under this specific start/goal configuration and is not an absolute guarantee")
    print(" of universal planning reliability across all arbitrary robot states.\n")

    clean_exit(0)


def main():
    benchmark_planners()


if __name__ == "__main__":
    main()
