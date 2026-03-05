cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include(FetchContent)
FetchContent_Declare(
  custom_mpl
  GIT_REPOSITORY https://github.com/asahin1/custom-mpl.git
  GIT_TAG main
)
FetchContent_MakeAvailable(custom_mpl)

find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(path_planning_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_utils REQUIRED)

# ------------------------------------------------------
# Library (contains node implementation only)
# ------------------------------------------------------
add_library(${PROJECT_NAME}
  src/image_map.cpp
  src/path_planner.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    tf2
    tf2_ros
    nav_msgs
    path_planning_interfaces
    robot_utils
)

target_link_libraries(${PROJECT_NAME}
  custom_mpl::custom_mpl
  ${OpenCV_LIBS}
  yaml-cpp
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
