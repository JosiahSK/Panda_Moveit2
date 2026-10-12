/**
 * cartesian_planner.cpp
 * Lesson 04 — Cartesian Waypoint Path in C++
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("panda_cartesian_planner_cpp");
static const std::string PLANNING_GROUP = "panda_arm";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("panda_cartesian_planner_cpp");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spin_thread = std::thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);

    // Waypoint 1: 15cm in X
    geometry_msgs::msg::Pose p1 = start_pose;
    p1.position.x += 0.15;
    waypoints.push_back(p1);

    // Waypoint 2: 10cm up in Z
    geometry_msgs::msg::Pose p2 = p1;
    p2.position.z += 0.10;
    waypoints.push_back(p2);

    // Waypoint 3: 15cm back in X
    geometry_msgs::msg::Pose p3 = p2;
    p3.position.x -= 0.15;
    waypoints.push_back(p3);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;        // 1cm step resolution
    const double jump_threshold = 0.0;   // 0 disables jump detection

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(LOGGER, "Cartesian path computed: %.1f%% achievable", fraction * 100.0);

    if (fraction > 0.90)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_WARN(LOGGER, "Path coverage too low (%.1f%%) to execute safely.", fraction * 100.0);
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
