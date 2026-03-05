#ifndef CABLE_PLAN_EXECUTOR_HPP
#define CABLE_PLAN_EXECUTOR_HPP

// Standard libraries
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

// ROS2 core
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// Messages
#include <lifecycle_msgs/msg/transition.hpp>

// Local libraries
#include "robot_utils/cable.hpp"

// Local messages
#include <cable_action_interfaces/action/cable_move.hpp>
#include <cable_action_interfaces/action/cables_hitch.hpp>

namespace cable_plan_executor {

class CablePlanExecutor : public rclcpp_lifecycle::LifecycleNode {
public:
  using JointPlan = std::vector<std::vector<robot_utils::cable::CableAction>>;
  using CableMoveAction = cable_action_interfaces::action::CableMove;
  using GoalHandleMove = rclcpp_action::ClientGoalHandle<CableMoveAction>;
  using CablesHitchAction = cable_action_interfaces::action::CablesHitch;
  using GoalHandleHitch = rclcpp_action::ClientGoalHandle<CablesHitchAction>;

  CablePlanExecutor();

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

private:
  // Parameters
  bool skip_hitches_;
  std::string plan_filepath_;
  std::vector<std::string> first_end_robot_names_;
  std::vector<std::string> last_end_robot_names_;
  double action_server_timeout_duration_;

  // Action Clients
  rclcpp_action::Client<CableMoveAction>::SharedPtr move_action_client_ptr_;
  rclcpp_action::Client<CablesHitchAction>::SharedPtr hitch_action_client_ptr_;

  // Planning variables
  std::vector<robot_utils::cable::Cable> cables_;
  JointPlan joint_plan_;
  std::vector<size_t> current_steps_;
  std::vector<int> hitch_coop_flags_;

  // Lifecycle management
  bool is_active_ = false;

  // Helper Methods
  void load_plan();

  // Action Request Generators
  CableMoveAction::Goal create_cable_move_action_goal(
      const robot_utils::cable::CableAction &move_action,
      const size_t cable_id);

  CablesHitchAction::Goal create_cables_hitch_action_goal(
      const robot_utils::cable::CableAction &hitch_action1,
      const size_t cable_id1,
      const robot_utils::cable::CableAction &hitch_action2,
      const size_t cable_id2);

  // Main Logic
  void send_next_goal(size_t cable_id);

  // Action Client Callbacks

  // Move
  void cable_move_goal_response_callback(
      const GoalHandleMove::SharedPtr &goal_handle, size_t cable_id);

  void cable_move_feedback_callback(
      GoalHandleMove::SharedPtr,
      const std::shared_ptr<const CableMoveAction::Feedback> feedback,
      size_t cable_id);

  void cable_move_result_callback(const GoalHandleMove::WrappedResult &result,
                                  size_t cable_id);

  // Hitch
  void cables_hitch_goal_response_callback(
      const GoalHandleHitch::SharedPtr &goal_handle, size_t cable_id1,
      size_t cable_id2);

  void cables_hitch_feedback_callback(
      GoalHandleHitch::SharedPtr,
      const std::shared_ptr<const CablesHitchAction::Feedback> feedback,
      size_t cable_id1, size_t cable_id2);

  void
  cables_hitch_result_callback(const GoalHandleHitch::WrappedResult &result,
                               size_t cable_id1, size_t cable_id2);
};
} // namespace cable_plan_executor

#endif