#ifndef ROBOT_UTILS_TF_LISTENER_WRAPPER_HPP
#define ROBOT_UTILS_TF_LISTENER_WRAPPER_HPP

#include "robot_utils/geometry.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf_utils.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace std::chrono_literals;

namespace robot_utils::tf {
class TfListenerWrapper {
public:
  explicit TfListenerWrapper(const rclcpp::Node::SharedPtr &node)
      : node_(node) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  bool try_get_transform_with_timeout(
      const std::string &target_frame, // map
      const std::string &source_frame, // robot
      geometry_msgs::msg::TransformStamped &out_transform,
      rclcpp::Duration timeout,
      const tf2::TimePoint &time = tf2::TimePointZero) const {
    auto start = node_->now();
    while (node_->now() - start < timeout) {
      try {
        out_transform = tf_buffer_->lookupTransform(target_frame, source_frame,
                                                    time, 100ms);
        return true;
      } catch (const tf2::TransformException &ex) {
        auto &clk = *node_->get_clock();
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), clk, 1000, "Could not transform %s to %s: %s",
            source_frame.c_str(), target_frame.c_str(), ex.what());
        std::this_thread::sleep_for(100ms);
      }
    }
    RCLCPP_ERROR(node_->get_logger(),
                 "Timeout while waiting for transform %s -> %s",
                 source_frame.c_str(), target_frame.c_str());
    return false;
  }

  bool
  try_get_transform(const std::string &target_frame, // map
                    const std::string &source_frame, // robot
                    geometry_msgs::msg::TransformStamped &out_transform,
                    const tf2::TimePoint &time = tf2::TimePointZero) const {
    try {
      out_transform =
          tf_buffer_->lookupTransform(target_frame, source_frame, time);
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_DEBUG(node_->get_logger(), "tf lookup failed: %s", ex.what());
      return false;
    }
  }

  ~TfListenerWrapper() = default;

  bool lookup_2D_pos_with_timeout(
      const std::string &target_frame, // map
      const std::string &source_frame, // robot
      robot_utils::geometry::Coords<double> &out_pose, rclcpp::Duration timeout,
      const tf2::TimePoint &time = tf2::TimePointZero) const {
    geometry_msgs::msg::TransformStamped tf_stamped;
    bool lookup_success = try_get_transform_with_timeout(
        target_frame, source_frame, tf_stamped, timeout, time);
    if (lookup_success) {
      out_pose = extract_position_2D(tf_stamped);
      return true;
    } else {
      return false;
    }
  }

  bool lookup_2D_pos(const std::string &target_frame, // map
                     const std::string &source_frame, // robot
                     robot_utils::geometry::Coords<double> &out_pose,
                     const tf2::TimePoint &time = tf2::TimePointZero) const {
    geometry_msgs::msg::TransformStamped tf_stamped;
    bool lookup_success =
        try_get_transform(target_frame, source_frame, tf_stamped, time);
    if (lookup_success) {
      out_pose = extract_position_2D(tf_stamped);
      return true;
    } else {
      return false;
    }
  }

  bool lookup_2D_pose_with_timeout(
      const std::string &target_frame, // map
      const std::string &source_frame, // robot
      robot_utils::geometry::Pose2D &out_pose, rclcpp::Duration timeout,
      const tf2::TimePoint &time = tf2::TimePointZero) const {
    geometry_msgs::msg::TransformStamped tf_stamped;
    bool lookup_success = try_get_transform_with_timeout(
        target_frame, source_frame, tf_stamped, timeout, time);
    if (lookup_success) {
      out_pose = extract_pose_2D(tf_stamped);
      return true;
    } else {
      return false;
    }
  }

  bool lookup_2D_pose(const std::string &target_frame, // map
                      const std::string &source_frame, // robot
                      robot_utils::geometry::Pose2D &out_pose,
                      const tf2::TimePoint &time = tf2::TimePointZero) const {
    geometry_msgs::msg::TransformStamped tf_stamped;
    bool lookup_success =
        try_get_transform(target_frame, source_frame, tf_stamped, time);
    if (lookup_success) {
      out_pose = extract_pose_2D(tf_stamped);
      return true;
    } else {
      return false;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
} // namespace robot_utils::tf

#endif