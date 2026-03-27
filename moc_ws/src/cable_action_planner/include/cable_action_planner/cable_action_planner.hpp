#ifndef CABLE_ACTION_PLANNER_HPP
#define CABLE_ACTION_PLANNER_HPP

// Standard libraries
#include <memory>

// ROS2 core
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Local libraries
#include "robot_utils/tf_listener_wrapper.hpp"

// Local messages
#include <cable_action_interfaces/action/cable_progress.hpp>
#include <cable_action_interfaces/action/cables_interlace.hpp>
#include <navigation_action_interfaces/action/follow_path.hpp>
#include <path_planning_interfaces/action/path_plan.hpp>
#include <robot_utils/cable.hpp>

namespace cable_action_planner {

class CableActionPlanner : public rclcpp::Node {
public:
  using CableProgressAction = cable_action_interfaces::action::CableProgress;
  using GoalHandleProgress =
      rclcpp_action::ServerGoalHandle<CableProgressAction>;
  using CablesInterlaceAction =
      cable_action_interfaces::action::CablesInterlace;
  using GoalHandleInterlace =
      rclcpp_action::ServerGoalHandle<CablesInterlaceAction>;

  using PathPlanningAction = path_planning_interfaces::action::PathPlan;
  using GoalHandlePathPlan =
      rclcpp_action::ClientGoalHandle<PathPlanningAction>;

  using FollowPathAction = navigation_action_interfaces::action::FollowPath;
  using GoalHandleFollowPath =
      rclcpp_action::ClientGoalHandle<FollowPathAction>;

  CableActionPlanner();
  ~CableActionPlanner();
  void initialize();
  int get_n_threads() const;

private:
  // Parameters
  int n_threads_;
  std::vector<std::string> rear_end_robot_names_;
  std::vector<std::string> front_end_robot_names_;
  double server_timeout_duration_;
  double tf_timeout_duration_;
  double path_planning_timeout_duration_;
  double communication_timeout_duration_;
  std::chrono::duration<double> communication_timeout_limit;
  std::chrono::duration<double> pp_timeout_limit;
  std::string target_frame_;
  double robot_radius_;

  double progress_last_wp_fallback_scale_default_;
  double progress_last_wp_fallback_scale_step_;
  double progress_last_wp_fallback_scale_max_;
  int progress_last_wp_fallback_attempts_max_;
  int progress_replan_attempts_max_;

  double interlace_circle_fallback_default_;
  double interlace_circle_fallback_step_;
  int interlace_circle_fallback_max_attempts_;

  double interlace_move_away_fallback_scale_default_;
  double interlace_move_away_fallback_scale_step_;
  int interlace_move_away_fallback_attempts_max_;

  std::map<rclcpp_action::GoalUUID, std::thread> active_threads_;
  std::mutex thread_mutex_;

  // tf
  std::unique_ptr<robot_utils::tf::TfListenerWrapper> tf_wrapper_;

  // Action Servers
  rclcpp_action::Server<CableProgressAction>::SharedPtr
      cable_progress_action_server_ptr_;
  rclcpp_action::Server<CablesInterlaceAction>::SharedPtr
      cables_interlace_action_server_ptr_;

  // Action Clients
  rclcpp_action::Client<PathPlanningAction>::SharedPtr
      path_planning_client_ptr_;
  std::unordered_map<std::string,
                     rclcpp_action::Client<FollowPathAction>::SharedPtr>
      follow_path_action_client_ptrs_;

  // Action plan variables
  std::vector<robot_utils::cable::Cable> cables_;
  std::vector<std::string> robot_names_;
  std::unordered_map<std::string, robot_utils::geometry::Coords<double>>
      robot_prev_cable_points_;
  std::unordered_map<std::string, bool> is_cables_first_progression;

  // Action Request Generators
  FollowPathAction::Goal create_follow_path_action_goal(
      const std::vector<geometry_msgs::msg::Point> &path);

  // Action Server Callbacks
  rclcpp_action::GoalResponse handle_cable_progress_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const CableProgressAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cable_progress_cancel(
      const std::shared_ptr<GoalHandleProgress> goal_handle);
  void handle_cable_progress_accepted(
      const std::shared_ptr<GoalHandleProgress> goal_handle);

  rclcpp_action::GoalResponse handle_cables_interlace_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const CablesInterlaceAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cables_interlace_cancel(
      const std::shared_ptr<GoalHandleInterlace> goal_handle);
  void handle_cables_interlace_accepted(
      const std::shared_ptr<GoalHandleInterlace> goal_handle);

  // Main Logic (Planning for Actions)
  void execute_next_cable_progress_step(
      const std::shared_ptr<GoalHandleProgress> goal_handle);
  void execute_next_cables_interlace_step(
      const std::shared_ptr<GoalHandleInterlace> goal_handle);

  // Path Following Action Client Callbacks
  void follow_path_feedback_callback(
      GoalHandleFollowPath::SharedPtr,
      const std::shared_ptr<const FollowPathAction::Feedback> feedback,
      const std::string &robot_name);
};

} // namespace cable_action_planner

#endif
