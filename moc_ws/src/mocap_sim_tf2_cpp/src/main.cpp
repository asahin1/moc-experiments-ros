// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// Local libraries
#include "mocap_sim_tf2_cpp/mocap_sim_tf2.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mocap_sim_tf2::MocapSimulator>());
  rclcpp::shutdown();
  return 0;
}