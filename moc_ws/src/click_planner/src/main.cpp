// ROS2 core
#include <rclcpp/rclcpp.hpp>

// Local libraries
#include "click_planner/click_planner.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<click_planner::ClickPlanner>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Unhandled exception: %s",
                 e.what());
    rclcpp::shutdown(); // Gracefully shut down ROS 2
    return 1;           // Non-zero exit to indicate error
  }
  rclcpp::shutdown();
  return 0;
}