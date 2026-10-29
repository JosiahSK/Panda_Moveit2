/**
 * scene_manager.cpp
 * Lesson 04 — Collision Objects & Planning Scene in C++
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thread>
#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("panda_scene_manager_cpp");
static const std::string PLANNING_GROUP = "panda_arm";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("panda_scene_manager_cpp");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spin_thread = std::thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // ── Create and add a box obstacle ──────────────────────────────
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "obstacle_box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.1, 0.4, 0.4};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.2;
    box_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface.applyCollisionObject(collision_object);
    RCLCPP_INFO(LOGGER, "Collision box 'obstacle_box' published to planning scene.");

    // Allow time for RViz and move_group to ingest the scene update
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Plan around the obstacle to named pose 'extended'
    move_group.setNamedTarget("extended");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Planning around obstacle: %s", success ? "SUCCESS" : "FAILED");
    if (success)
    {
        move_group.execute(plan);
    }

    // Clean up obstacle from the planning scene
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::vector<std::string> object_ids = {"obstacle_box"};
    planning_scene_interface.removeCollisionObjects(object_ids);
    RCLCPP_INFO(LOGGER, "Removed obstacle from planning scene.");

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
