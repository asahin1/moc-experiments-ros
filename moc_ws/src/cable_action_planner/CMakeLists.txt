cmake_minimum_required(VERSION 3.8)
project(cable_action_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cable_action_interfaces REQUIRED)
find_package(path_planning_interfaces REQUIRED)
find_package(navigation_action_interfaces REQUIRED)
find_package(robot_utils REQUIRED)

# ------------------------------------------------------
# Library (contains node implementation only)
# ------------------------------------------------------
add_library(${PROJECT_NAME}
  src/cable_action_planner.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_action
    geometry_msgs
    cable_action_interfaces
    path_planning_interfaces
    navigation_action_interfaces
    robot_utils
)

# ------------------------------------------------------
# Executable (contains main())
# ------------------------------------------------------
add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

# ------------------------------------------------------
# Installation
# ------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)


ament_package()
