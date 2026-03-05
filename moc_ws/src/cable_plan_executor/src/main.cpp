// ROS2 core
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

// Local libraries
#include "cable_plan_executor/cable_plan_executor.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<cable_plan_executor::CablePlanExecutor>();
    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(node->get_node_base_interface());
    exe.spin();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Unhandled exception: %s",
                 e.what());
    rclcpp::shutdown(); // Gracefully shut down ROS 2
    return 1;           // Non-zero exit to indicate error
  }
  rclcpp::shutdown();
  return 0;
}