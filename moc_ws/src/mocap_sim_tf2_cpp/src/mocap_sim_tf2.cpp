// Standard libraries
#include <memory>

// ROS2 Core
#include <rclcpp/duration.hpp>

// Third-party
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Local libraries
#include "mocap_sim_tf2_cpp/mocap_sim_tf2.hpp"

using namespace std::chrono_literals;

namespace mocap_sim_tf2 {
MocapSimulator::MocapSimulator() : Node("mocap_sim_tf2") {
  generator_ = std::default_random_engine(rd_());

  robot_names_ = this->declare_parameter<std::vector<std::string>>(
      "robot_names", std::vector<std::string>{});
  noise_mean_ = this->declare_parameter<double>("noise_mean", 0.0);
  noise_stddev_ = this->declare_parameter<double>("noise_stddev", 0.0);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  for (size_t i{0}; i < robot_names_.size(); ++i) {
    std::ostringstream stream;
    std::string robot_name = robot_names_[i];
    stream << "/" << robot_name.c_str() << "/pose";
    std::string topic_name = stream.str();
    auto subscription_callback =
        [this,
         robot_name](const geometry_msgs::msg::Pose::SharedPtr msg) -> void {
      handle_car_pose(msg, robot_name);
    };
    subscriptions_.push_back(
        this->create_subscription<geometry_msgs::msg::Pose>(
            topic_name, 10, subscription_callback));
  }
}

void MocapSimulator::handle_car_pose(
    const std::shared_ptr<geometry_msgs::msg::Pose> msg,
    const std::string &car_name) {
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = car_name.c_str();

  std::normal_distribution<double> distribution(noise_mean_, noise_stddev_);
  std::vector<double> noise_samples;
  int num_samples = 3;
  for (int i = 0; i < num_samples; ++i) {
    noise_samples.push_back(distribution(generator_));
  }

  t.transform.translation.x = msg->position.x + noise_samples[0];
  t.transform.translation.y = msg->position.y + noise_samples[1];
  t.transform.translation.z = msg->position.z;

  tf2::Quaternion tf2_quat;
  tf2::convert(msg->orientation, tf2_quat);
  tf2::Matrix3x3 m(tf2_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  double theta = yaw + noise_samples[2];
  theta = fmod(theta, 2 * M_PI);
  if (theta < 0) {
    theta += 2 * M_PI;
  }
  // RCLCPP_INFO(this->get_logger(), "Broadcasting %s tf: (%f, %f, %f)",
  // car_name.c_str(), t.transform.translation.x,
  //             t.transform.translation.y, theta);
  tf2_quat.setRPY(0, 0, theta);
  t.transform.rotation = tf2::toMsg(tf2_quat);

  tf_broadcaster_->sendTransform(t);
}

} // namespace mocap_sim_tf2