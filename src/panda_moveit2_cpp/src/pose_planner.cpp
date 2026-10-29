/**
 * pose_planner.cpp
 * Lesson 04 — Cartesian Pose Goal Planning in C++
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("panda_pose_planner_cpp");
static const std::string PLANNING_GROUP = "panda_arm";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "panda_pose_planner_cpp",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spin_thread = std::thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(5.0);

    // Target Cartesian Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.307;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.590;
    target_pose.orientation.x = 0.924;
    target_pose.orientation.y = -0.383;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Planning to target Cartesian pose: %s", success ? "SUCCESS" : "FAILED");

    if (success)
    {
        RCLCPP_INFO(LOGGER, "Executing planned trajectory...");
        move_group.execute(plan);
    }

    move_group.clearPoseTargets();
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
