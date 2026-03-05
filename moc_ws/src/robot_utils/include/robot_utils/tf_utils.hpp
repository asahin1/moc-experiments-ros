#ifndef ROBOT_UTILS_TF_UTILS_HPP
#define ROBOT_UTILS_TF_UTILS_HPP

#include "robot_utils/geometry.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.hpp>

namespace robot_utils::tf {

inline geometry::Coords<double>
extract_position_2D(const geometry_msgs::msg::TransformStamped &tf) {
  geometry::Coords<double> p;
  p.x = tf.transform.translation.x;
  p.y = tf.transform.translation.y;
  return p;
}

inline geometry::Pose2D
extract_pose_2D(const geometry_msgs::msg::TransformStamped &tf) {
  geometry::Pose2D pose;
  pose.position.x = tf.transform.translation.x;
  pose.position.y = tf.transform.translation.y;
  double roll, pitch, yaw;

  tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  pose.heading.x = std::cos(yaw);
  pose.heading.y = std::sin(yaw);
  return pose;
}
} // namespace robot_utils::tf

#endif