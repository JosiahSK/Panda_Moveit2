/**
 * basic_motion_planner.cpp
 * Lesson 04 — Basic joint-space motion planning in C++
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <cmath>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("panda_cpp_planner");
static const std::string PLANNING_GROUP = "panda_arm";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "panda_motion_planner_cpp",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spin_thread = std::thread([&executor]() { executor.spin(); });

    // ── Create MoveGroupInterface ─────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(5.0);

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "EEF link: %s", move_group.getEndEffectorLink().c_str());

    // ── Plan to named state 'ready' ───────────────────────────────────
    move_group.setNamedTarget("ready");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan to 'ready': %s", success ? "SUCCESS" : "FAILED");
    if (success) { move_group.execute(plan); }

    // ── Plan to custom joint values ───────────────────────────────────
    std::vector<double> joints = {
        0.0,
        -M_PI / 4.0,
        0.0,
        -M_PI / 2.0,
        0.0,
        M_PI / 3.0,
        M_PI / 4.0
    };
    move_group.setJointValueTarget(joints);
    success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Custom joint plan: %s", success ? "SUCCESS" : "FAILED");
    if (success) { move_group.execute(plan); }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
