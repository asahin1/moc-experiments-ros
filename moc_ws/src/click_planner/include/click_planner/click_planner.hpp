#ifndef CLICK_PLANNER_HPP
#define CLICK_PLANNER_HPP

// Standard libraries
#include <memory>

// ROS2 core
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Local libraries
#include "robot_utils/tf_listener_wrapper.hpp"

// Local messages=
#include <navigation_action_interfaces/action/follow_path.hpp>
#include <path_planning_interfaces/srv/path_plan.hpp>
#include <std_msgs/msg/string.hpp>

namespace click_planner {
class ClickPlanner : public rclcpp::Node {
public:
  using PathPlanningService = path_planning_interfaces::srv::PathPlan;

  using FollowPathAction = navigation_action_interfaces::action::FollowPath;
  using GoalHandleFollowPath =
      rclcpp_action::ClientGoalHandle<FollowPathAction>;

  ClickPlanner();
  void initialize();

private:
  // Parameters
  std::vector<std::string> robot_names_;
  std::string selected_robot_;
  double server_timeout_duration_;
  double tf_timeout_duration_;
  std::string target_frame_;
  double robot_radius_;

  // tf
  std::unique_ptr<robot_utils::tf::TfListenerWrapper> tf_wrapper_;

  // Clicked point subscription
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      clicked_pt_sub_ptr_;

  // Selected robot subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      selected_robot_sub_ptr_;

  // Service Clients
  rclcpp::Client<PathPlanningService>::SharedPtr path_planning_client_ptr_;

  // Action Clients
  std::unordered_map<std::string,
                     rclcpp_action::Client<FollowPathAction>::SharedPtr>
      follow_path_action_client_ptrs_;

  // Robot selection callback
  void robot_selection_callback(const std_msgs::msg::String &msg);

  // Clicked point callback
  void click_callback(const geometry_msgs::msg::PointStamped &msg);

  void path_planning_response_cb(
      rclcpp::Client<PathPlanningService>::SharedFuture future,
      const std::string &robot_name);

  FollowPathAction::Goal create_follow_path_action_goal(
      const std::vector<geometry_msgs::msg::Point> &path);

  void follow_path_goal_response_callback(
      const GoalHandleFollowPath::SharedPtr &fp_goal_handle,
      const std::string &robot_name);

  void follow_path_feedback_callback(
      GoalHandleFollowPath::SharedPtr,
      const std::shared_ptr<const FollowPathAction::Feedback> feedback,
      const std::string &robot_name);

  void
  follow_path_result_callback(const GoalHandleFollowPath::WrappedResult &result,
                              const std::string &robot_name);
};
} // namespace click_planner

#endif
