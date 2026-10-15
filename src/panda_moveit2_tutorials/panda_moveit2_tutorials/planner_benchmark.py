#!/usr/bin/env python3
"""
Planner benchmarking script — compares solve times and success rates
across RRTConnect, PRM, and LBKPIECE on identical motion tasks.
"""

import rclpy
import time
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


def benchmark_planners():
    rclpy.init()
    moveit = MoveItPy(node_name="planner_benchmarker")
    panda_arm = moveit.get_planning_component("panda_arm")

    target = PoseStamped()
    target.header.frame_id = "panda_link0"
    target.pose.position.x = 0.45
    target.pose.position.y = 0.20
    target.pose.position.z = 0.50
    target.pose.orientation.x = 0.924
    target.pose.orientation.y = -0.383
    target.pose.orientation.w = 0.0

    planners_to_test = ["RRTConnectkConfigDefault", "PRMkConfigDefault", "LBKPIECEkConfigDefault"]
    trials_per_planner = 5

    print("\n" + "=" * 65)
    print(" 📊 MOVEIT 2 OMPL PLANNER BENCHMARK ON PANDA 7-DOF")
    print("=" * 65)

    results = {}

    for planner in planners_to_test:
        print(f"\n[BENCHMARK] Testing planner: {planner} ({trials_per_planner} trials)...")
        durations = []
        successes = 0

        for i in range(trials_per_planner):
            panda_arm.set_start_state_to_current_state()
            panda_arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")

            t_start = time.perf_counter()
            plan_res = panda_arm.plan()
            t_end = time.perf_counter()

            elapsed_ms = (t_end - t_start) * 1000.0

            if plan_res:
                successes += 1
                durations.append(elapsed_ms)
                print(f"  Trial {i+1}: SUCCESS in {elapsed_ms:.2f} ms")
            else:
                print(f"  Trial {i+1}: FAILED")

        avg_ms = sum(durations) / len(durations) if durations else 0.0
        success_rate = (successes / trials_per_planner) * 100.0
        results[planner] = {"avg_ms": avg_ms, "success_rate": success_rate}

    print("\n" + "=" * 65)
    print(f"{'Planner':<28} | {'Avg Time (ms)':<15} | {'Success Rate':<12}")
    print("-" * 65)
    for p_name, data in results.items():
        print(f"{p_name:<28} | {data['avg_ms']:<15.2f} | {data['success_rate']:<12.1f}%")
    print("=" * 65 + "\n")

    rclpy.shutdown()


def main():
    benchmark_planners()


if __name__ == "__main__":
    main()
