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
#include <cable_action_interfaces/action/cable_move.hpp>
#include <cable_action_interfaces/action/cables_hitch.hpp>
#include <navigation_action_interfaces/action/follow_path.hpp>
#include <path_planning_interfaces/action/path_plan.hpp>
#include <robot_utils/cable.hpp>

namespace cable_action_planner {

class CableActionPlanner : public rclcpp::Node {
public:
  using CableMoveAction = cable_action_interfaces::action::CableMove;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<CableMoveAction>;
  using CablesHitchAction = cable_action_interfaces::action::CablesHitch;
  using GoalHandleHitch = rclcpp_action::ServerGoalHandle<CablesHitchAction>;

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
  std::vector<std::string> first_end_robot_names_;
  std::vector<std::string> last_end_robot_names_;
  double server_timeout_duration_;
  double tf_timeout_duration_;
  double path_planning_timeout_duration_;
  std::string target_frame_;
  double robot_radius_;

  double move_last_wp_fallback_scale_default_;
  double move_last_wp_fallback_scale_step_;
  double move_last_wp_fallback_scale_max_;
  int move_last_wp_fallback_attempts_max_;
  int move_replan_attempts_max_;

  double hitch_circle_fallback_default_;
  double hitch_circle_fallback_step_;
  int hitch_circle_fallback_max_attempts_;

  double hitch_move_away_fallback_scale_default_;
  double hitch_move_away_fallback_scale_step_;
  double hitch_move_away_fallback_scale_max_;
  int hitch_move_away_fallback_attempts_max_;

  std::map<rclcpp_action::GoalUUID, std::thread> active_threads_;
  std::mutex thread_mutex_;

  // tf
  std::unique_ptr<robot_utils::tf::TfListenerWrapper> tf_wrapper_;

  // Action Servers
  rclcpp_action::Server<CableMoveAction>::SharedPtr
      cable_move_action_server_ptr_;
  rclcpp_action::Server<CablesHitchAction>::SharedPtr
      cables_hitch_action_server_ptr_;

  // Action Clients
  rclcpp_action::Client<PathPlanningAction>::SharedPtr
      path_planning_client_ptr_;
  std::unordered_map<std::string,
                     rclcpp_action::Client<FollowPathAction>::SharedPtr>
      follow_path_action_client_ptrs_;

  // Action plan variables
  std::vector<robot_utils::cable::Cable> cables_;
  std::vector<std::string> robot_names_;

  // Action Request Generators
  FollowPathAction::Goal create_follow_path_action_goal(
      const std::vector<geometry_msgs::msg::Point> &path);

  // Action Server Callbacks
  rclcpp_action::GoalResponse
  handle_cable_move_goal(const rclcpp_action::GoalUUID &uuid,
                         std::shared_ptr<const CableMoveAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cable_move_cancel(const std::shared_ptr<GoalHandleMove> goal_handle);
  void
  handle_cable_move_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);

  rclcpp_action::GoalResponse
  handle_cables_hitch_goal(const rclcpp_action::GoalUUID &uuid,
                           std::shared_ptr<const CablesHitchAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cables_hitch_cancel(
      const std::shared_ptr<GoalHandleHitch> goal_handle);
  void handle_cables_hitch_accepted(
      const std::shared_ptr<GoalHandleHitch> goal_handle);

  // Main Logic (Planning for Actions)
  void execute_next_cable_move_step(
      const std::shared_ptr<GoalHandleMove> goal_handle);
  void execute_next_cables_hitch_step(
      const std::shared_ptr<GoalHandleHitch> goal_handle);

  // Path Following Action Client Callbacks
  void follow_path_feedback_callback(
      GoalHandleFollowPath::SharedPtr,
      const std::shared_ptr<const FollowPathAction::Feedback> feedback,
      const std::string &robot_name);
};

} // namespace cable_action_planner

#endif
