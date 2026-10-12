cmake_minimum_required(VERSION 3.8)
project(panda_moveit2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

set(MOVEIT_DEPS
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_core
  geometry_msgs
  moveit_msgs
  shape_msgs
)

# ── Executables ──────────────────────────────────────────────────────
add_executable(basic_motion_planner_cpp src/basic_motion_planner.cpp)
ament_target_dependencies(basic_motion_planner_cpp ${MOVEIT_DEPS})
target_include_directories(basic_motion_planner_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(pose_planner_cpp src/pose_planner.cpp)
ament_target_dependencies(pose_planner_cpp ${MOVEIT_DEPS})

add_executable(cartesian_planner_cpp src/cartesian_planner.cpp)
ament_target_dependencies(cartesian_planner_cpp ${MOVEIT_DEPS})

add_executable(scene_manager_cpp src/scene_manager.cpp)
ament_target_dependencies(scene_manager_cpp ${MOVEIT_DEPS})

# ── Install ──────────────────────────────────────────────────────────
install(TARGETS
  basic_motion_planner_cpp
  pose_planner_cpp
  cartesian_planner_cpp
  scene_manager_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
