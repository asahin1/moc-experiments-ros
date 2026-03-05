// Standard libraries
#include <algorithm>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <memory>

// ROS2 Core
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <robot_utils/geometry.hpp>

// Local libraries
#include "click_planner/click_planner.hpp"
#include "robot_utils/log_utils.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace click_planner {

ClickPlanner::ClickPlanner() : Node("click_planner") {
  n_threads_ = this->declare_parameter<int>("n_threads", 4);
  robot_names_ = this->declare_parameter<std::vector<std::string>>(
      "robot_names", std::vector<std::string>{});
  server_timeout_duration_ =
      this->declare_parameter<double>("server_timeout", 5.0);
  tf_timeout_duration_ = this->declare_parameter<double>("tf_timeout", 5.0);
  path_planning_timeout_duration_ =
      this->declare_parameter<double>("path_planner_timeout", 5.0);
  target_frame_ = this->declare_parameter<std::string>(
      "target_frame", ""); // Follow path frame_id
  robot_radius_ = this->declare_parameter<double>("robot_radius", 0.15);

  auto callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  clicked_pt_sub_ptr_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "clicked_point", 10,
          std::bind(&ClickPlanner::click_callback, this, _1), sub_options);

  selected_robot_sub_ptr_ = this->create_subscription<std_msgs::msg::String>(
      "selected_robot", 10,
      std::bind(&ClickPlanner::robot_selection_callback, this, _1));

  std::chrono::duration<double> timeout_duration(server_timeout_duration_);

  path_planning_client_ptr_ =
      rclcpp_action::create_client<PathPlanningAction>(this, "plan_path");
  RCLCPP_INFO(this->get_logger(), "Waiting for path planning server...");
  if (!this->path_planning_client_ptr_->wait_for_action_server(
          timeout_duration)) {
    throw std::runtime_error(
        "Path planning action server not available after wait");
  }
  RCLCPP_INFO(this->get_logger(), "Path planning server found.");

  // Action client handles
  for (size_t i{0}; i < robot_names_.size(); ++i) {
    follow_path_action_client_ptrs_.insert(
        {robot_names_[i], rclcpp_action::create_client<FollowPathAction>(
                              this, robot_names_[i] + "/follow_path_action")});
  }

  RCLCPP_INFO(this->get_logger(), "Waiting for follow path action servers...");
  for (const auto &robot : robot_names_) {
    if (!this->follow_path_action_client_ptrs_[robot]->wait_for_action_server(
            timeout_duration)) {
      throw std::runtime_error(
          "Follow path action servers not available after wait");
    }
  }
  RCLCPP_INFO(this->get_logger(), "Follow path action servers found.");
}

void ClickPlanner::initialize() {
  RCLCPP_INFO(this->get_logger(), "Creating tf wrapper.");
  tf_wrapper_ = std::make_unique<robot_utils::tf::TfListenerWrapper>(
      this->shared_from_this());
  RCLCPP_INFO(this->get_logger(), "tf wrapper created.");
}

int ClickPlanner::get_n_threads() const { return n_threads_; }

// Action Request Generators
ClickPlanner::FollowPathAction::Goal
ClickPlanner::create_follow_path_action_goal(
    const std::vector<geometry_msgs::msg::Point> &path) {
  FollowPathAction::Goal goal_msg;
  std::vector<geometry_msgs::msg::PointStamped> stamped_path;
  for (const auto &pt : path) {
    geometry_msgs::msg::PointStamped stamped_pt;
    stamped_pt.header.frame_id = target_frame_;
    stamped_pt.point.x = pt.x;
    stamped_pt.point.y = pt.y;
    stamped_path.push_back(stamped_pt);
  }
  goal_msg.path = stamped_path;
  return goal_msg;
}

void ClickPlanner::robot_selection_callback(const std_msgs::msg::String &msg) {
  std::string cand = msg.data;
  if (std::find(robot_names_.begin(), robot_names_.end(), cand) ==
      robot_names_.end()) {
    RCLCPP_WARN(this->get_logger(),
                "Selected robot name  %s is not in robot names list.",
                cand.c_str());
    return;
  }
  selected_robot_ = msg.data;
}

void ClickPlanner::click_callback(const geometry_msgs::msg::PointStamped &msg) {
  if (selected_robot_.empty()) {
    RCLCPP_WARN(
        this->get_logger(),
        "No valid robot is selected yet, cannot send the clicked goal.");
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "%s Read target - (%f, %f)",
               robot_utils::log::get_robot_prefix(selected_robot_).c_str(),
               msg.point.x, msg.point.y);

  std::chrono::duration<double> pp_communication_timeout_limit(
      path_planning_timeout_duration_);
  std::chrono::duration<double> pp_timeout_limit(
      path_planning_timeout_duration_);

  // Send async path planning request
  geometry_msgs::msg::Point goal_pt;
  goal_pt.x = msg.point.x;
  goal_pt.y = msg.point.y;

  auto request = PathPlanningAction::Goal();
  request.robot_name = selected_robot_;
  request.goal = goal_pt;

  auto send_goal_options =
      rclcpp_action::Client<PathPlanningAction>::SendGoalOptions();

  RCLCPP_INFO(this->get_logger(), "%s Requesting path plan to point (%f, %f)",
              robot_utils::log::get_robot_prefix(selected_robot_).c_str(),
              goal_pt.x, goal_pt.y);

  auto pp_goal_future =
      path_planning_client_ptr_->async_send_goal(request, send_goal_options);

  RCLCPP_INFO(this->get_logger(), "Comm timeout limit: %f",
              path_planning_timeout_duration_);

  if (pp_goal_future.wait_for(pp_communication_timeout_limit) !=
      std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Timeout waiting for path planning request "
                                    "to be accepted.");
    return;
  }

  auto pp_goal_handle = pp_goal_future.get();
  if (!pp_goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Path planning request rejected.");
    return;
  }

  bool planning_success = false;
  auto result_future =
      path_planning_client_ptr_->async_get_result(pp_goal_handle);
  auto status = result_future.wait_for(pp_timeout_limit);
  GoalHandlePathPlan::WrappedResult pp_wrapped_result;

  if (status == std::future_status::timeout) {
    RCLCPP_INFO(this->get_logger(), "Timeout reached. Cancelling path planning "
                                    "goal.");
    auto cancel_future =
        path_planning_client_ptr_->async_cancel_goal(pp_goal_handle);
    return;
  } else if (status == std::future_status::ready) {
    pp_wrapped_result = result_future.get();
    if (pp_wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      planning_success = true;
    }
  }
  if (planning_success) {
    auto path = pp_wrapped_result.result->path;
    // Send async follow path request
    auto action_goal = create_follow_path_action_goal(path);

    RCLCPP_DEBUG(this->get_logger(), "%s Sending follow-path action goal",
                 robot_utils::log::get_robot_prefix(selected_robot_).c_str());

    rclcpp_action::Client<FollowPathAction>::SendGoalOptions opts;
    opts.feedback_callback =
        std::bind(&ClickPlanner::follow_path_feedback_callback, this, _1, _2,
                  selected_robot_);
    auto fp_handle_future =
        follow_path_action_client_ptrs_[selected_robot_]->async_send_goal(
            action_goal, opts);
    auto fp_goal_handle = fp_handle_future.get();
    auto fp_result_future =
        follow_path_action_client_ptrs_[selected_robot_]->async_get_result(
            fp_goal_handle);
    auto fp_wrapped_result = fp_result_future.get();
  }
}

// Path Following Action Client Callbacks

void ClickPlanner::follow_path_feedback_callback(
    GoalHandleFollowPath::SharedPtr,
    const std::shared_ptr<const FollowPathAction::Feedback> feedback,
    const std::string &robot_name) {
  RCLCPP_DEBUG(this->get_logger(),
               "%s Feedback received. Remaining path points: %d",
               robot_utils::log::get_robot_prefix(robot_name).c_str(),
               feedback->remaining_path_points);
}

} // namespace click_planner
