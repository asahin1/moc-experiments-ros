// Standard libraries
#include <random>

// ROS2 core
#include <rclcpp/rclcpp.hpp>

// Third-party
#include <tf2_ros/transform_broadcaster.hpp>

// Messages
#include <geometry_msgs/msg/pose.hpp>

namespace mocap_sim_tf2 {
class MocapSimulator : public rclcpp::Node {
public:
  MocapSimulator();

private:
  // Parameters
  double noise_mean_;
  double noise_stddev_;
  std::vector<std::string> robot_names_;

  // Subscriptions;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr>
      subscriptions_;

  // tf
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Random helpers
  std::random_device rd_;                // Non-deterministic source for seeding
  std::default_random_engine generator_; // Seed the engine

  void handle_car_pose(const std::shared_ptr<geometry_msgs::msg::Pose> msg,
                       const std::string &car_name);
};
} // namespace mocap_sim_tf2
