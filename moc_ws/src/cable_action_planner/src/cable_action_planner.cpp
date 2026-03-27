// Standard libraries
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <future>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <memory>

// ROS2 Core
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/create_client.hpp>
#include <robot_utils/cable.hpp>
#include <robot_utils/geometry.hpp>
#include <std_msgs/msg/detail/int16__struct.hpp>

// Local libraries
#include "cable_action_planner/cable_action_planner.hpp"
#include "robot_utils/log_utils.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace cable_action_planner {

CableActionPlanner::CableActionPlanner() : Node("cable_action_planner") {
  // Declare parameters
  n_threads_ = this->declare_parameter<int>("n_threads", 4);
  rear_end_robot_names_ = this->declare_parameter<std::vector<std::string>>(
      "rear_end_robot_names", std::vector<std::string>{});
  front_end_robot_names_ = this->declare_parameter<std::vector<std::string>>(
      "front_end_robot_names", std::vector<std::string>{});
  server_timeout_duration_ =
      this->declare_parameter<double>("server_timeout", 5.0);
  tf_timeout_duration_ = this->declare_parameter<double>("tf_timeout", 5.0);
  path_planning_timeout_duration_ =
      this->declare_parameter<double>("path_planner_timeout", 5.0);
  communication_timeout_duration_ =
      this->declare_parameter<double>("communication_timeout", 5.0);
  target_frame_ = this->declare_parameter<std::string>(
      "target_frame", ""); // Follow path frame_id

  robot_radius_ = this->declare_parameter<double>("robot_radius", 0.15);
  progress_last_wp_fallback_scale_default_ = this->declare_parameter<double>(
      "progress_last_wp_fallback_scale_default", 0.15);
  progress_last_wp_fallback_scale_step_ = this->declare_parameter<double>(
      "progress_last_wp_fallback_scale_step", 0.15);
  progress_last_wp_fallback_scale_max_ = this->declare_parameter<double>(
      "progress_last_wp_fallback_scale_max", 0.5);
  progress_last_wp_fallback_attempts_max_ =
      this->declare_parameter<int>("progress_last_wp_fallback_attempts_max", 5);
  progress_replan_attempts_max_ =
      this->declare_parameter<int>("progress_replan_attempts_max", 5);
  interlace_circle_fallback_default_ = this->declare_parameter<double>(
      "interlace_circle_fallback_default", 0.15);
  interlace_circle_fallback_step_ =
      this->declare_parameter<double>("interlace_circle_fallback_step", 0.15);
  interlace_circle_fallback_max_attempts_ =
      this->declare_parameter<int>("interlace_circle_fallback_max_attempts", 5);
  interlace_move_away_fallback_scale_default_ = this->declare_parameter<double>(
      "interlace_move_away_fallback_scale_default", 0.15);
  interlace_move_away_fallback_scale_step_ = this->declare_parameter<double>(
      "interlace_move_away_fallback_scale_step", 0.15);
  interlace_move_away_fallback_attempts_max_ = this->declare_parameter<int>(
      "interlace_move_away_fallback_attempts_max", 5);

  // Timeout durations
  communication_timeout_limit =
      std::chrono::duration<double>(communication_timeout_duration_);
  pp_timeout_limit =
      std::chrono::duration<double>(path_planning_timeout_duration_);

  // Generate Cables vector
  if (rear_end_robot_names_.size() != front_end_robot_names_.size()) {
    throw std::runtime_error(
        "Attempting to use " + std::to_string(rear_end_robot_names_.size()) +
        " rear end robots with " +
        std::to_string(front_end_robot_names_.size()) + " front end robots. ");
  }
  for (size_t i{0}; i < rear_end_robot_names_.size(); ++i) {
    cables_.push_back(robot_utils::cable::Cable{"cable" + std::to_string(i),
                                                rear_end_robot_names_[i],
                                                front_end_robot_names_[i]});
  }
  for (size_t i{0}; i < rear_end_robot_names_.size(); ++i) {
    robot_names_.push_back(rear_end_robot_names_[i]);
  }
  for (size_t i{0}; i < front_end_robot_names_.size(); ++i) {
    robot_names_.push_back(front_end_robot_names_[i]);
  }

  std::chrono::duration<double> timeout_duration(server_timeout_duration_);

  path_planning_client_ptr_ =
      rclcpp_action::create_client<PathPlanningAction>(this, "plan_path");

  RCLCPP_INFO(this->get_logger(), "Waiting for path planning action server...");
  if (!this->path_planning_client_ptr_->wait_for_action_server(
          timeout_duration)) {
    throw std::runtime_error(
        "Path planning action server not available after wait");
  }
  RCLCPP_INFO(this->get_logger(), "Path planning server found.");

  for (size_t i{0}; i < robot_names_.size(); ++i) {
    follow_path_action_client_ptrs_.insert(
        {robot_names_[i], rclcpp_action::create_client<FollowPathAction>(
                              this, robot_names_[i] + "/follow_path_action")});
    robot_prev_cable_points_.insert({robot_names_[i], robot_utils::geometry::Coords<double>{0.0, 0.0}});
    is_cables_first_progression.insert({robot_names_[i], true});
  }
  RCLCPP_INFO(this->get_logger(), "Waiting for follow path action servers...");
  for (const auto &robot : robot_names_) {
    if (!this->follow_path_action_client_ptrs_[robot]->wait_for_action_server(
            timeout_duration)) {
      throw std::runtime_error(
          "Follow path action servers not available after wait");
    }
  }
  RCLCPP_INFO(this->get_logger(), "Follow path action server found.");

  this->cable_progress_action_server_ptr_ =
      rclcpp_action::create_server<CableProgressAction>(
          this, "cable_progress_action",
          std::bind(&CableActionPlanner::handle_cable_progress_goal, this, _1,
                    _2),
          std::bind(&CableActionPlanner::handle_cable_progress_cancel, this,
                    _1),
          std::bind(&CableActionPlanner::handle_cable_progress_accepted, this,
                    _1));

  this->cables_interlace_action_server_ptr_ =
      rclcpp_action::create_server<CablesInterlaceAction>(
          this, "cables_interlace_action",
          std::bind(&CableActionPlanner::handle_cables_interlace_goal, this, _1,
                    _2),
          std::bind(&CableActionPlanner::handle_cables_interlace_cancel, this,
                    _1),
          std::bind(&CableActionPlanner::handle_cables_interlace_accepted, this,
                    _1));

  RCLCPP_INFO(this->get_logger(),
              "Cable Progress and Interlace Action servers initialized.");
}

CableActionPlanner::~CableActionPlanner() {
  std::lock_guard<std::mutex> lock(thread_mutex_);
  for (auto &[uuid, thread] : active_threads_) {
    if (thread.joinable()) {
      thread.join(); // Block until the thread finishes its current loop
    }
  }
}

void CableActionPlanner::initialize() {
  RCLCPP_DEBUG(this->get_logger(), "Creating tf wrapper.");
  tf_wrapper_ = std::make_unique<robot_utils::tf::TfListenerWrapper>(
      this->shared_from_this());
  RCLCPP_DEBUG(this->get_logger(), "tf wrapper created.");
}

int CableActionPlanner::get_n_threads() const { return n_threads_; }

// Action Request Generators
CableActionPlanner::FollowPathAction::Goal
CableActionPlanner::create_follow_path_action_goal(
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

// Action Server Callbacks
rclcpp_action::GoalResponse CableActionPlanner::handle_cable_progress_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const CableProgressAction::Goal> goal) {
  RCLCPP_INFO(
      this->get_logger(), "%s Received Cable Progress action request",
      robot_utils::log::get_cable_prefix(cables_[goal->cable_id]).c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CableActionPlanner::handle_cable_progress_cancel(
    const std::shared_ptr<GoalHandleProgress> goal_handle) {
  auto goal = goal_handle->get_goal();
  size_t cable_id = goal->cable_id;
  RCLCPP_INFO(this->get_logger(),
              "%s Received request to cancel Cable Progress action",
              robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str());
  std::string robot_name = front_end_robot_names_[cable_id];
  follow_path_action_client_ptrs_[robot_name]->async_cancel_all_goals();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CableActionPlanner::handle_cable_progress_accepted(
    const std::shared_ptr<GoalHandleProgress> goal_handle) {
  std::lock_guard<std::mutex> lock(thread_mutex_);
  const auto goal_id = goal_handle->get_goal_id();
  // this needs to return quickly to avoid blocking the executor
  RCLCPP_INFO(this->get_logger(),
              "%s Cable Progression action request accepted.",
              robot_utils::log::get_cable_prefix(
                  cables_[goal_handle->get_goal()->cable_id])
                  .c_str());
  active_threads_[goal_id] = std::thread(
      std::bind(&CableActionPlanner::execute_next_cable_progress_step, this,
                std::placeholders::_1),
      goal_handle);
}

rclcpp_action::GoalResponse CableActionPlanner::handle_cables_interlace_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const CablesInterlaceAction::Goal> goal) {
  RCLCPP_INFO(
      this->get_logger(), "%s-%s Received Cables Interlace action request",
      robot_utils::log::get_cable_prefix(cables_[goal->cable_id1]).c_str(),
      robot_utils::log::get_cable_prefix(cables_[goal->cable_id2]).c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
CableActionPlanner::handle_cables_interlace_cancel(
    const std::shared_ptr<GoalHandleInterlace> goal_handle) {
  auto goal = goal_handle->get_goal();
  size_t cable_id1 = goal->cable_id1;
  size_t cable_id2 = goal->cable_id2;
  auto cable_end1 = static_cast<robot_utils::cable::CableEnd>(goal->cable_end1);
  auto cable_end2 = static_cast<robot_utils::cable::CableEnd>(goal->cable_end2);
  RCLCPP_INFO(this->get_logger(),
              "%s-%s Received request to cancel Cables Interlace action",
              robot_utils::log::get_cable_prefix(cables_[cable_id1]).c_str(),
              robot_utils::log::get_cable_prefix(cables_[cable_id2]).c_str());
  std::string robot1_name =
      robot_utils::cable::get_robot_at_end(cables_[cable_id1], cable_end1);
  std::string robot2_name =
      robot_utils::cable::get_robot_at_end(cables_[cable_id2], cable_end2);
  follow_path_action_client_ptrs_[robot1_name]->async_cancel_all_goals();
  follow_path_action_client_ptrs_[robot2_name]->async_cancel_all_goals();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CableActionPlanner::handle_cables_interlace_accepted(
    const std::shared_ptr<GoalHandleInterlace> goal_handle) {
  std::lock_guard<std::mutex> lock(thread_mutex_);
  const auto goal_id = goal_handle->get_goal_id();
  // this needs to return quickly to avoid blocking the executor
  auto goal = goal_handle->get_goal();
  size_t cable_id1 = goal->cable_id1;
  size_t cable_id2 = goal->cable_id2;
  RCLCPP_INFO(this->get_logger(),
              "%s-%s Cables Interlace action request accepted.",
              robot_utils::log::get_cable_prefix(cables_[cable_id1]).c_str(),
              robot_utils::log::get_cable_prefix(cables_[cable_id2]).c_str());
  active_threads_[goal_id] = std::thread(
      std::bind(&CableActionPlanner::execute_next_cables_interlace_step, this,
                std::placeholders::_1),
      goal_handle);
}

// Main Logic (Planning for Actions)

void CableActionPlanner::execute_next_cable_progress_step(
    const std::shared_ptr<GoalHandleProgress> goal_handle) {
  // Read goal info
  auto goal = goal_handle->get_goal();
  std::string robot_name = front_end_robot_names_[goal->cable_id];
  std::vector<geometry_msgs::msg::Point> waypoints = goal->waypoints;

  // Initialize state machine
  size_t index = 1; // 0-th waypoint is the current point so we can start from 1
  double progress_last_wp_fallback = progress_last_wp_fallback_scale_default_;
  int progress_last_wp_fallback_attempts = 0;
  int progress_replan_attempts = 0;

  // Only if it is the first progression, assign the rear robot's last cable point as the first waypoint (indexed 1)
  if (is_cables_first_progression[robot_name]) {
    robot_prev_cable_points_[rear_end_robot_names_[goal->cable_id]] = robot_utils::geometry::Coords<double>(waypoints[1].x, waypoints[1].y);
    is_cables_first_progression[robot_name] = false;
    is_cables_first_progression[rear_end_robot_names_[goal->cable_id]] = false;
  }


  // Initialize emtpy result and feedback
  auto result = std::make_shared<CableProgressAction::Result>();
  auto feedback = std::make_shared<CableProgressAction::Feedback>();

  // Execution loop
  while (index < waypoints.size() && rclcpp::ok()) {
    // Check cancellation
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "%s Cable Progress action cancelling.",
                  robot_utils::log::get_robot_prefix(robot_name).c_str());
      goal_handle->canceled(result);
      return;
    }

    // Action feedback
    feedback->remaining_waypoints = waypoints.size() - index;
    goal_handle->publish_feedback(feedback);

    // Action logic by steps
    geometry_msgs::msg::Point goal_pt;
    // Stop short if last waypoint
    if (index == waypoints.size() - 1) {
      auto last_wp_geo = waypoints[index];
      robot_utils::geometry::Coords<double> last_wp{last_wp_geo.x,
                                                    last_wp_geo.y};
      auto prev_wp_geo = waypoints[index - 1];
      robot_utils::geometry::Coords<double> prev_wp{prev_wp_geo.x,
                                                    prev_wp_geo.y};
      robot_utils::geometry::Coords<double> v = prev_wp - last_wp;
      double scale =
          std::min(progress_last_wp_fallback_scale_max_,
                   progress_last_wp_fallback * robot_radius_ / v.length());
      last_wp = last_wp + v * scale;
      goal_pt.x = last_wp.x;
      goal_pt.y = last_wp.y;
    } else {
      goal_pt = waypoints[index];
      // Assign moving robots last cable point as the one before last waypoint
      robot_prev_cable_points_[robot_name] = robot_utils::geometry::Coords<double>(goal_pt.x, goal_pt.y);
    }

    // Send async path planning request
    auto request = PathPlanningAction::Goal();
    request.robot_name = robot_name;
    request.goal = goal_pt;

    auto send_goal_options =
        rclcpp_action::Client<PathPlanningAction>::SendGoalOptions();

    RCLCPP_DEBUG(this->get_logger(),
                 "%s Requesting path plan to waypoint (%zu/%zu)",
                 robot_utils::log::get_robot_prefix(robot_name).c_str(), index,
                 waypoints.size() - 1);

    auto pp_goal_future =
        path_planning_client_ptr_->async_send_goal(request, send_goal_options);

    if (pp_goal_future.wait_for(communication_timeout_limit) !=
        std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(),
                   "%s Timeout waiting for path planning request "
                   "to be accepted. Aborting cable progression action",
                   robot_utils::log::get_robot_prefix(robot_name).c_str());
      goal_handle->abort(result);
    }

    auto pp_goal_handle = pp_goal_future.get();
    if (!pp_goal_handle) {
      RCLCPP_ERROR(this->get_logger(),
                   "%s Path planning request rejected. Aborting cable "
                   "progression action",
                   robot_utils::log::get_robot_prefix(robot_name).c_str());
      goal_handle->abort(result);
    }

    bool planning_success = false;
    auto result_future =
        path_planning_client_ptr_->async_get_result(pp_goal_handle);
    auto status = result_future.wait_for(pp_timeout_limit);
    GoalHandlePathPlan::WrappedResult pp_wrapped_result;

    if (status == std::future_status::timeout) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "%s Timeout reached. Cancelling path planning "
          "goal. Will apply fallback logic for cable progression action.",
          robot_utils::log::get_robot_prefix(robot_name).c_str());
      auto cancel_future =
          path_planning_client_ptr_->async_cancel_goal(pp_goal_handle);
      if (cancel_future.wait_for(communication_timeout_limit) ==
          std::future_status::ready) {
        auto cancel_response = cancel_future.get();

        if (cancel_response->return_code !=
            action_msgs::srv::CancelGoal_Response::ERROR_NONE) {
          RCLCPP_WARN(this->get_logger(),
                      "%s Path planning server rejected the cancel request!",
                      robot_utils::log::get_robot_prefix(robot_name).c_str());
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "%s Path planning server acknowledged cancel request. "
                      "Waiting for "
                      "termination...",
                      robot_utils::log::get_robot_prefix(robot_name).c_str());
        }
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "%s Path planning server failed to acknowledge cancel "
                    "request. It might be dead.",
                    robot_utils::log::get_robot_prefix(robot_name).c_str());
      }
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
                   robot_utils::log::get_robot_prefix(robot_name).c_str());

      rclcpp_action::Client<FollowPathAction>::SendGoalOptions opts;
      opts.feedback_callback =
          std::bind(&CableActionPlanner::follow_path_feedback_callback, this,
                    _1, _2, robot_name);
      auto fp_handle_future =
          follow_path_action_client_ptrs_[robot_name]->async_send_goal(
              action_goal, opts);
      if (fp_handle_future.wait_for(communication_timeout_limit) !=
          std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(),
                     "%s Timeout waiting for follow path request "
                     "to be accepted. Aborting cable progression action",
                     robot_utils::log::get_robot_prefix(robot_name).c_str());
        goal_handle->abort(result);
      }
      auto fp_goal_handle = fp_handle_future.get();
      if (!fp_goal_handle) {
        RCLCPP_ERROR(this->get_logger(),
                     "%s Follow path request rejected. "
                     "Cancelling cable progression action",
                     robot_utils::log::get_robot_prefix(robot_name).c_str());
        goal_handle->abort(result);
      }

      auto fp_result_future =
          follow_path_action_client_ptrs_[robot_name]->async_get_result(
              fp_goal_handle);
      std::future_status fp_status;
      do {
        fp_status = fp_result_future.wait_for(100ms);
        if (!rclcpp::ok()) {
          return;
        }
        if (goal_handle->is_canceling()) {
          follow_path_action_client_ptrs_[robot_name]->async_cancel_goal(
              fp_goal_handle);
          goal_handle->canceled(result);
          return;
        }
      } while (fp_status != std::future_status::ready);
      auto fp_wrapped_result = fp_result_future.get();
      if (pp_wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        index++;
      }
    } else {
      bool max_attempts_reached = false;
      if (index == waypoints.size() - 1) {
        if (progress_last_wp_fallback_attempts <
            progress_last_wp_fallback_attempts_max_) {
          progress_last_wp_fallback += progress_last_wp_fallback_scale_step_;
          progress_last_wp_fallback_attempts++;
          RCLCPP_DEBUG(this->get_logger(),
                       "%s will try to repeat the last progression action step "
                       "with modified "
                       "progress_last_wp_fallback: %f,  (%d/%d attempts).",
                       robot_utils::log::get_robot_prefix(robot_name).c_str(),
                       progress_last_wp_fallback,
                       progress_last_wp_fallback_attempts,
                       progress_last_wp_fallback_attempts_max_);
        } else {
          max_attempts_reached = true;
        }
      } else {
        if (progress_replan_attempts < progress_replan_attempts_max_) {
          progress_replan_attempts++;
          RCLCPP_DEBUG(
              this->get_logger(),
              "%s will try to replan for the last progression action step,  "
              "(%d/%d attempts).",
              robot_utils::log::get_robot_prefix(robot_name).c_str(),
              progress_replan_attempts, progress_replan_attempts_max_);
        } else {
          max_attempts_reached = true;
        }
      }
      if (max_attempts_reached) {
        RCLCPP_ERROR(
            this->get_logger(),
            "%s Could not find a way to perform cable progression action after "
            "attempted fallbacks. Aborting...",
            robot_utils::log::get_robot_prefix(robot_name).c_str());
        goal_handle->abort(result);
        return;
      }
    }
  }

  // Check if done
  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "%s Cable Progression action succeeded.",
                robot_utils::log::get_robot_prefix(robot_name).c_str());
    return;
  }
}

void CableActionPlanner::execute_next_cables_interlace_step(
    const std::shared_ptr<GoalHandleInterlace> goal_handle) {
  // Read goal info
  auto goal = goal_handle->get_goal();
  std::string robot1_name = robot_utils::cable::get_robot_at_end(
      cables_[goal->cable_id1],
      static_cast<robot_utils::cable::CableEnd>(goal->cable_end1));
  std::string robot2_name = robot_utils::cable::get_robot_at_end(
      cables_[goal->cable_id2],
      static_cast<robot_utils::cable::CableEnd>(goal->cable_end2));

  // Timeouts
  std::chrono::duration<double> pp_communication_timeout_limit(
      path_planning_timeout_duration_);
  std::chrono::duration<double> pp_timeout_limit(
      path_planning_timeout_duration_);
  std::chrono::duration<double> fp_communication_timeout_limit(
      path_planning_timeout_duration_);
  rclcpp::Duration robot_pose_timeout =
      rclcpp::Duration::from_seconds(tf_timeout_duration_);

  // Read robot1's position with timeout
  robot_utils::geometry::Coords<double> robot1_pos;
  bool robot1_pose_read = tf_wrapper_->lookup_2D_pos_with_timeout(
      target_frame_, robot1_name, robot1_pos, robot_pose_timeout);
  if (!robot1_pose_read) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for transform");
    throw std::runtime_error(
        "Cannot read robot1 pose from tf in time for interlace planning");
  }
  RCLCPP_DEBUG(this->get_logger(), "Robot1 pos read: (%f, %f)", robot1_pos.x,
               robot1_pos.y);

  // Read robot2's pose with timeout
  robot_utils::geometry::Coords<double> robot2_pos;
  bool robot2_pose_read = tf_wrapper_->lookup_2D_pos_with_timeout(
      target_frame_, robot2_name, robot2_pos, robot_pose_timeout);
  if (!robot2_pose_read) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for transform");
    throw std::runtime_error(
        "Cannot read robot2 pose from tf in time for interlace planning");
  }
  RCLCPP_DEBUG(this->get_logger(), "Robot2 pos read: (%f, %f)", robot2_pos.x,
               robot2_pos.y);
  RCLCPP_DEBUG(this->get_logger(), "Robot1 exit: (%f, %f)", goal->exit_point1.x,
               goal->exit_point1.y);
  RCLCPP_DEBUG(this->get_logger(), "Robot2 exit: (%f, %f)", goal->exit_point2.x,
               goal->exit_point2.y);


  robot_utils::geometry::Coords<double> prev_pt1;
  robot_utils::geometry::Coords<double> prev_pt2;
  // if it is robot's first progression front robot must be interlacing, we can use the rear robots position as the previous cable point
  std::string rear_robot1_name_;
  if(is_cables_first_progression[robot1_name]) {
    assert(static_cast<robot_utils::cable::CableEnd>(goal->cable_end1) == robot_utils::cable::CableEnd::Front);
    rear_robot1_name_ = rear_end_robot_names_[goal->cable_id1];
    // Read rear_robot1's position with timeout
    robot_utils::geometry::Coords<double> rear_robot1_pos;
    bool rear_robot1_pose_read = tf_wrapper_->lookup_2D_pos_with_timeout(
        target_frame_, rear_robot1_name_, rear_robot1_pos, robot_pose_timeout);
    if (!rear_robot1_pose_read) {
      RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for transform");
      throw std::runtime_error(
          "Cannot read rear_robot1 pose from tf in time for interlace planning");
    }
    RCLCPP_DEBUG(this->get_logger(), "Rear Robot1 pos read: (%f, %f)", rear_robot1_pos.x,
                rear_robot1_pos.y);
    prev_pt1 = rear_robot1_pos;
  }
  else{
    prev_pt1 = robot_prev_cable_points_[robot1_name];
  }
  std::string rear_robot2_name_;
  if(is_cables_first_progression[robot2_name]) {
    assert(static_cast<robot_utils::cable::CableEnd>(goal->cable_end2) == robot_utils::cable::CableEnd::Front);
    rear_robot2_name_ = rear_end_robot_names_[goal->cable_id2];
    // Read rear_robot2's pose with timeout
    robot_utils::geometry::Coords<double> rear_robot2_pos;
    bool rear_robot2_pose_read = tf_wrapper_->lookup_2D_pos_with_timeout(
        target_frame_, rear_robot2_name_, rear_robot2_pos, robot_pose_timeout);
    if (!rear_robot2_pose_read) {
      RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for transform");
      throw std::runtime_error(
          "Cannot read rear_robot2 pose from tf in time for interlace planning");
    }
    RCLCPP_DEBUG(this->get_logger(), "Rear Robot2 pos read: (%f, %f)", rear_robot2_pos.x,
                rear_robot2_pos.y);
    prev_pt2 = rear_robot2_pos;
  }
  else{
    prev_pt2 = robot_prev_cable_points_[robot2_name];
  }

  // Initialize state machine
  size_t step = 0;
  double interlace_circle_fallback = interlace_circle_fallback_default_;
  double interlace_move_away_fallback =
      interlace_move_away_fallback_scale_default_;
  int interlace_circle_attempts = 0;
  int interlace_move_away_attempts = 0;
  bool reverse_circle = false;

  // Initialize empty result and feedback
  auto result = std::make_shared<CablesInterlaceAction::Result>();
  auto feedback = std::make_shared<CablesInterlaceAction::Feedback>();

  while (step < 4 && rclcpp::ok()) {
    // Check cancellation
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(),
                  "%s-%s Cables Interlace action cancelling.",
                  robot_utils::log::get_robot_prefix(robot1_name).c_str(),
                  robot_utils::log::get_robot_prefix(robot2_name).c_str());
      goal_handle->canceled(result);
      return;
    }

    // Action feedback
    feedback->current_interlace_step = step;
    goal_handle->publish_feedback(feedback);

    // 3 Steps to the interlace: 1) robot2 rotates around robot1 (path to the
    // same point with some signature) 2) robot1 crosses over, 3) robot2 crosses
    // over

    // Switch logic should determine the moving_robot_name and the waypoint
    std::string moving_robot_name;
    std::string other_robot_name;
    robot_utils::geometry::Coords<double> waypoint;
    robot_utils::geometry::Coords<double> in_point1{robot1_pos.x, robot1_pos.y};
    robot_utils::geometry::Coords<double> in_point2{robot2_pos.x, robot2_pos.y};
    robot_utils::geometry::Coords<double> exit_point1{goal->exit_point1.x,
                                                      goal->exit_point1.y};
    robot_utils::geometry::Coords<double> exit_point2{goal->exit_point2.x,
                                                      goal->exit_point2.y};
    robot_utils::geometry::Coords<double> vertex_coord{goal->vertex_coords.x,
                                                       goal->vertex_coords.y};


    // double signed_angle = robot_utils::geometry::signed_angle(
    //     in_point2 - in_point1, exit_point2 - in_point1);
    // double signed_angle = robot_utils::geometry::signed_angle(
    //     exit_point1 - in_point1, exit_point2 - in_point2);
    double signed_angle = robot_utils::geometry::signed_angle(
        prev_pt2 - in_point2, prev_pt2 - in_point1);
    int rotation_dir = 1;
    if (signed_angle > 0) {
      rotation_dir = -1;
    }
    double move_away_scale1 = interlace_move_away_fallback;
    double move_away_scale2 = interlace_move_away_fallback;
    int8_t h_sig{0};
    switch (step) {
    case 0:
      // Move robot1 in place (go back to this step in case of fallbacks)
      moving_robot_name = robot1_name;
      waypoint =
          in_point1 + (vertex_coord - in_point1) * interlace_circle_fallback;
      break;
    case 1:
      if (reverse_circle) {
        moving_robot_name = robot1_name;
        other_robot_name = robot2_name;
        robot_utils::geometry::Coords<double> v_out = prev_pt1 - robot2_pos;
        robot_utils::geometry::Coords<double> v_between = in_point1 - robot2_pos;
        double dist = v_between.length();
        waypoint = robot2_pos + v_out.normalized() * dist;
        // waypoint = in_point1;
        h_sig = -rotation_dir;
      } else {
        moving_robot_name = robot2_name;
        other_robot_name = robot1_name;
        robot_utils::geometry::Coords<double> v_out = prev_pt2 - robot1_pos;
        robot_utils::geometry::Coords<double> v_between = in_point2 - robot1_pos;
        double dist = v_between.length();
        waypoint = robot1_pos + v_out.normalized() * dist;
        // waypoint = in_point2;
        h_sig = rotation_dir;
      }
      break;
    case 2: {
      if (reverse_circle) {
        moving_robot_name = robot2_name;
        robot_utils::geometry::Coords<double> v = exit_point2 - in_point2;
        robot_utils::geometry::Coords<double> v_unit = v * (1 / v.length());
        double scale = v.length();
        if (!goal->is_2last) {
          scale = std::min(v.length(), move_away_scale2 * robot_radius_);
        }
        waypoint = in_point2 + v_unit * scale;
      } else {
        moving_robot_name = robot1_name;
        robot_utils::geometry::Coords<double> v = exit_point1 - in_point1;
        robot_utils::geometry::Coords<double> v_unit = v * (1 / v.length());
        double scale = v.length();
        if (!goal->is_1last) {
          scale = std::min(v.length(), move_away_scale1 * robot_radius_);
        }
        waypoint = in_point1 + v_unit * scale;
      }
      robot_prev_cable_points_[moving_robot_name] = waypoint;
      break;
    }
    case 3: {
      if (reverse_circle) {
        moving_robot_name = robot1_name;
        robot_utils::geometry::Coords<double> v = exit_point1 - in_point1;
        robot_utils::geometry::Coords<double> v_unit = v * (1 / v.length());
        double scale = v.length();
        if (!goal->is_1last) {
          scale = std::min(v.length(), move_away_scale1 * robot_radius_);
        }
        waypoint = in_point1 + v_unit * scale;
      } else {
        moving_robot_name = robot2_name;
        robot_utils::geometry::Coords<double> v = exit_point2 - in_point2;
        robot_utils::geometry::Coords<double> v_unit = v * (1 / v.length());
        double scale = v.length();
        if (!goal->is_2last) {
          scale = std::min(v.length(), move_away_scale2 * robot_radius_);
        }
        waypoint = in_point2 + v_unit * scale;
      }
      robot_prev_cable_points_[moving_robot_name] = waypoint;
      break;
    }
    }
    RCLCPP_DEBUG(this->get_logger(), "Interlace wp (%zu/3): (%f, %f)", step,
                 waypoint.x, waypoint.y);

    // Send async path planning request
    auto request = PathPlanningAction::Goal();
    request.robot_name = moving_robot_name;
    geometry_msgs::msg::Point waypoint_gm;
    waypoint_gm.x = waypoint.x;
    waypoint_gm.y = waypoint.y;
    request.goal = waypoint_gm;
    request.other_robot_name = other_robot_name;
    request.h_sig = h_sig;

    auto send_goal_options =
        rclcpp_action::Client<PathPlanningAction>::SendGoalOptions();

    RCLCPP_DEBUG(
        this->get_logger(),
        "%s Requesting path plan for interlace construction step (%zu/3) - "
        "with other robot: %s",
        robot_utils::log::get_robot_prefix(moving_robot_name).c_str(), step,
        other_robot_name.c_str());

    auto pp_goal_future =
        path_planning_client_ptr_->async_send_goal(request, send_goal_options);

    if (pp_goal_future.wait_for(pp_communication_timeout_limit) !=
        std::future_status::ready) {
      RCLCPP_ERROR(
          this->get_logger(),
          "%s Timeout waiting for path planning request "
          "to be accepted. Aborting cable interlace action",
          robot_utils::log::get_robot_prefix(moving_robot_name).c_str());
      goal_handle->abort(result);
    }

    auto pp_goal_handle = pp_goal_future.get();
    if (!pp_goal_handle) {
      RCLCPP_ERROR(
          this->get_logger(),
          "%s Path planning request rejected. Aborting cable interlace action",
          robot_utils::log::get_robot_prefix(moving_robot_name).c_str());
      goal_handle->abort(result);
    }

    bool planning_success = false;
    auto result_future =
        path_planning_client_ptr_->async_get_result(pp_goal_handle);
    auto status = result_future.wait_for(pp_timeout_limit);
    GoalHandlePathPlan::WrappedResult pp_wrapped_result;

    if (status == std::future_status::timeout) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "%s Timeout reached. Cancelling path planning "
          "goal. Will apply interlace fallback logic.",
          robot_utils::log::get_robot_prefix(moving_robot_name).c_str());
      auto cancel_future =
          path_planning_client_ptr_->async_cancel_goal(pp_goal_handle);
      if (cancel_future.wait_for(pp_communication_timeout_limit) ==
          std::future_status::ready) {
        auto cancel_response = cancel_future.get();

        if (cancel_response->return_code !=
            action_msgs::srv::CancelGoal_Response::ERROR_NONE) {
          RCLCPP_WARN(
              this->get_logger(),
              "%s Path planning server rejected the cancel request!",
              robot_utils::log::get_robot_prefix(moving_robot_name).c_str());
        } else {
          RCLCPP_INFO(
              this->get_logger(),
              "%s Path planning server acknowledged cancel request. "
              "Waiting for "
              "termination...",
              robot_utils::log::get_robot_prefix(moving_robot_name).c_str());
        }
      } else {
        RCLCPP_WARN(
            this->get_logger(),
            "%s Path planning server failed to acknowledge cancel "
            "request. It might be dead.",
            robot_utils::log::get_robot_prefix(moving_robot_name).c_str());
      }
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

      RCLCPP_DEBUG(
          this->get_logger(), "%s Sending follow-path action goal",
          robot_utils::log::get_robot_prefix(moving_robot_name).c_str());

      rclcpp_action::Client<FollowPathAction>::SendGoalOptions opts;
      opts.feedback_callback =
          std::bind(&CableActionPlanner::follow_path_feedback_callback, this,
                    _1, _2, moving_robot_name);
      auto fp_handle_future =
          follow_path_action_client_ptrs_[moving_robot_name]->async_send_goal(
              action_goal, opts);
      if (fp_handle_future.wait_for(fp_communication_timeout_limit) !=
          std::future_status::ready) {
        RCLCPP_ERROR(
            this->get_logger(),
            "%s Timeout waiting for follow path request "
            "to be accepted. Aborting cable interlace action",
            robot_utils::log::get_robot_prefix(moving_robot_name).c_str());
        goal_handle->abort(result);
      }
      auto fp_goal_handle = fp_handle_future.get();
      if (!fp_goal_handle) {
        RCLCPP_ERROR(
            this->get_logger(),
            "%s Follow path request rejected. Cancelling cable "
            "interlace action",
            robot_utils::log::get_robot_prefix(moving_robot_name).c_str());
        goal_handle->abort(result);
      }

      auto fp_result_future =
          follow_path_action_client_ptrs_[moving_robot_name]->async_get_result(
              fp_goal_handle);
      std::future_status fp_status;
      do {
        fp_status = fp_result_future.wait_for(100ms);
        if (!rclcpp::ok()) {
          return;
        }
        if (goal_handle->is_canceling()) {
          follow_path_action_client_ptrs_[moving_robot_name]->async_cancel_goal(
              fp_goal_handle);
          goal_handle->canceled(result);
          return;
        }
      } while (fp_status != std::future_status::ready);
      auto fp_wrapped_result = fp_result_future.get();
      if (pp_wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        step++;
      }
    } else {
      int updated_step = step;
      switch (step) {
      case 1:
        if (!reverse_circle) {
          reverse_circle = true;
          break;
        } else {
          updated_step--;
          [[fallthrough]];
        }
      case 0:
        if (interlace_circle_attempts <
            interlace_circle_fallback_max_attempts_) {
          interlace_circle_fallback += interlace_circle_fallback_step_;
          interlace_circle_attempts++;
          reverse_circle = false;
        } else {
          RCLCPP_ERROR(
              this->get_logger(),
              "%s Could not find a way to perform cable interlace action after "
              "attempted fallbacks. Aborting...",
              robot_utils::log::get_robot_prefix(moving_robot_name).c_str());
          goal_handle->abort(result);
        }
        break;
      case 2:
      case 3:
        if (interlace_move_away_attempts <
            interlace_move_away_fallback_attempts_max_) {
          interlace_move_away_fallback +=
              interlace_move_away_fallback_scale_step_;
          interlace_move_away_attempts++;
        }
        break;
      }
      step = updated_step;
      RCLCPP_DEBUG(
          this->get_logger(),
          "%s will try to repeat the last interlace action step with modified "
          "variables.",
          robot_utils::log::get_robot_prefix(moving_robot_name).c_str());
    }
  }

  // Check if done
  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "%s-%s Cables Interlace Goal succeeded",
                robot_utils::log::get_robot_prefix(robot1_name).c_str(),
                robot_utils::log::get_robot_prefix(robot2_name).c_str());
    return;
  }
}

// Path Following Action Client Callbacks

void CableActionPlanner::follow_path_feedback_callback(
    GoalHandleFollowPath::SharedPtr,
    const std::shared_ptr<const FollowPathAction::Feedback> feedback,
    const std::string &robot_name) {
  RCLCPP_DEBUG(this->get_logger(),
               "%s Feedback received. Remaining path points: %d",
               robot_utils::log::get_robot_prefix(robot_name).c_str(),
               feedback->remaining_path_points);
}
} // namespace cable_action_planner
