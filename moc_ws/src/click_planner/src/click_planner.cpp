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

  robot_names_ = this->declare_parameter<std::vector<std::string>>(
      "robot_names", std::vector<std::string>{});
  server_timeout_duration_ =
      this->declare_parameter<double>("server_timeout", 5.0);
  tf_timeout_duration_ = this->declare_parameter<double>("tf_timeout", 5.0);
  target_frame_ = this->declare_parameter<std::string>(
      "target_frame", ""); // Follow path frame_id
  robot_radius_ = this->declare_parameter<double>("robot_radius", 0.15);

  clicked_pt_sub_ptr_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "clicked_point", 10,
          std::bind(&ClickPlanner::click_callback, this, _1));

  selected_robot_sub_ptr_ = this->create_subscription<std_msgs::msg::String>(
      "selected_robot", 10,
      std::bind(&ClickPlanner::robot_selection_callback, this, _1));

  std::chrono::duration<double> timeout_duration(server_timeout_duration_);

  path_planning_client_ptr_ =
      this->create_client<PathPlanningService>("plan_path");
  RCLCPP_INFO(this->get_logger(), "Waiting for path planning server...");
  if (!path_planning_client_ptr_->wait_for_service(timeout_duration)) {
    throw std::runtime_error("Path planning server not available after wait");
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

  geometry_msgs::msg::Point goal_pt;
  goal_pt.x = msg.point.x;
  goal_pt.y = msg.point.y;

  // Send async path planning request
  auto request = std::make_shared<PathPlanningService::Request>();
  request->robot_name = selected_robot_;
  request->goal = goal_pt;

  RCLCPP_INFO(this->get_logger(), "%s Requesting path plan to point (%f, %f)",
              robot_utils::log::get_robot_prefix(selected_robot_).c_str(),
              goal_pt.x, goal_pt.y);

  auto future = path_planning_client_ptr_->async_send_request(
      request,
      [this](rclcpp::Client<PathPlanningService>::SharedFuture future) {
        this->path_planning_response_cb(future, selected_robot_);
      });
}

void ClickPlanner::path_planning_response_cb(
    rclcpp::Client<PathPlanningService>::SharedFuture future,
    const std::string &robot_name) {

  if (!rclcpp::ok()) {
    return;
  }

  auto response = future.get();
  if (!response) {
    RCLCPP_WARN(this->get_logger(),
                "%s Path planning response could not be received.",
                robot_utils::log::get_robot_prefix(robot_name).c_str());
  }
  if (!response->success) {
    RCLCPP_WARN(this->get_logger(), "%s Path planning failed.",
                robot_utils::log::get_robot_prefix(robot_name).c_str());
    return;
  }

  auto path = response->path;

  // Send async follow path request
  auto action_goal = create_follow_path_action_goal(path);

  RCLCPP_DEBUG(this->get_logger(), "%s Sending follow-path action goal",
               robot_utils::log::get_robot_prefix(robot_name).c_str());

  rclcpp_action::Client<FollowPathAction>::SendGoalOptions opts;

  opts.goal_response_callback = std::bind(
      &ClickPlanner::follow_path_goal_response_callback, this, _1, robot_name);

  opts.feedback_callback = std::bind(
      &ClickPlanner::follow_path_feedback_callback, this, _1, _2, robot_name);

  opts.result_callback = std::bind(&ClickPlanner::follow_path_result_callback,
                                   this, _1, robot_name);

  follow_path_action_client_ptrs_[robot_name]->async_send_goal(action_goal,
                                                               opts);
}

// Path Following Action Client Callbacks

void ClickPlanner::follow_path_goal_response_callback(
    const GoalHandleFollowPath::SharedPtr &fp_goal_handle,
    const std::string &robot_name) {
  if (!fp_goal_handle) {
    RCLCPP_ERROR(this->get_logger(),
                 "%s Follow Path Goal was rejected by server",
                 robot_utils::log::get_robot_prefix(robot_name).c_str());
  } else {
    RCLCPP_DEBUG(this->get_logger(),
                 "%s Follow Path Goal accepted by server, waiting for result",
                 robot_utils::log::get_robot_prefix(robot_name).c_str());
  }
}

void ClickPlanner::follow_path_feedback_callback(
    GoalHandleFollowPath::SharedPtr,
    const std::shared_ptr<const FollowPathAction::Feedback> feedback,
    const std::string &robot_name) {
  RCLCPP_DEBUG(this->get_logger(),
               "%s Feedback received. Remaining path points: %d",
               robot_utils::log::get_robot_prefix(robot_name).c_str(),
               feedback->remaining_path_points);
}

void ClickPlanner::follow_path_result_callback(
    const GoalHandleFollowPath::WrappedResult &result,
    const std::string &robot_name) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_DEBUG(this->get_logger(), "Follow-path action SUCCEEDED");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_WARN(this->get_logger(), "Follow-path action ABORTED");
    break;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_WARN(this->get_logger(), "Follow-path action CANCELED");
    break;
  case rclcpp_action::ResultCode::UNKNOWN:
    RCLCPP_ERROR(this->get_logger(), "Follow-path action UNKNOWN");
    break;
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "%s Follow-path action failed",
                 robot_utils::log::get_robot_prefix(robot_name).c_str());
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "%s Follow Path Goal reached.",
               robot_utils::log::get_robot_prefix(robot_name).c_str());
}
} // namespace click_planner
