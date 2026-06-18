cmake_minimum_required(VERSION 3.8)
project(semantic_nav_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(semantic_nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/validate_semantic.cpp
  src/escalate_to_llm_recovery.cpp
  src/path_clear_condition.cpp
  src/query_semantic_context.cpp
  src/register_nodes.cpp
  src/emit_obstacle_signal.cpp
  src/operator_prompt.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name}
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  semantic_nav_interfaces
  geometry_msgs
  builtin_interfaces
  nav_msgs
  std_msgs
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bt_plugins test/test_bt_plugins.cpp)
  target_include_directories(test_bt_plugins PRIVATE include)
  target_link_libraries(test_bt_plugins ${library_name})
  ament_target_dependencies(test_bt_plugins
    rclcpp
    nav2_behavior_tree
    behaviortree_cpp_v3
    semantic_nav_interfaces
    geometry_msgs
    builtin_interfaces
    nav_msgs
    std_msgs
  )
endif()

ament_package()
