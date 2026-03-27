cmake_minimum_required(VERSION 3.8)
project(go2_jump_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(go2_jump_core REQUIRED)
find_package(go2_jump_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(jump_intent_node src/jump_intent_node.cpp)
target_compile_features(jump_intent_node PRIVATE cxx_std_17)
ament_target_dependencies(jump_intent_node
  go2_jump_core
  go2_jump_msgs
  rclcpp
)

install(
  TARGETS jump_intent_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
