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
#include <cable_action_interfaces/action/cable_progress.hpp>
#include <cable_action_interfaces/action/cables_interlace.hpp>

namespace cable_plan_executor {

class CablePlanExecutor : public rclcpp_lifecycle::LifecycleNode {
public:
  using JointPlan = std::vector<std::vector<robot_utils::cable::CableAction>>;
  using CableProgressAction = cable_action_interfaces::action::CableProgress;
  using GoalHandleProgress =
      rclcpp_action::ClientGoalHandle<CableProgressAction>;
  using CablesInterlaceAction =
      cable_action_interfaces::action::CablesInterlace;
  using GoalHandleInterlace =
      rclcpp_action::ClientGoalHandle<CablesInterlaceAction>;

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
  bool skip_interlaces_;
  std::string plan_filepath_;
  std::vector<std::string> rear_end_robot_names_;
  std::vector<std::string> front_end_robot_names_;
  double action_server_timeout_duration_;

  // Action Clients
  rclcpp_action::Client<CableProgressAction>::SharedPtr
      progress_action_client_ptr_;
  rclcpp_action::Client<CablesInterlaceAction>::SharedPtr
      interlace_action_client_ptr_;

  // Planning variables
  std::vector<robot_utils::cable::Cable> cables_;
  JointPlan joint_plan_;
  std::vector<size_t> current_steps_;
  std::vector<int> interlace_coop_flags_;

  // Lifecycle management
  bool is_active_ = false;

  // Helper Methods
  void load_plan();

  // Action Request Generators
  CableProgressAction::Goal create_cable_progress_action_goal(
      const robot_utils::cable::CableAction &progress_action,
      const size_t cable_id);

  CablesInterlaceAction::Goal create_cables_interlace_action_goal(
      const robot_utils::cable::CableAction &interlace_action1,
      const size_t cable_id1,
      const robot_utils::cable::CableAction &interlace_action2,
      const size_t cable_id2);

  // Main Logic
  void send_next_goal(size_t cable_id);

  // Action Client Callbacks

  // Progress
  void cable_progress_goal_response_callback(
      const GoalHandleProgress::SharedPtr &goal_handle, size_t cable_id);

  void cable_progress_feedback_callback(
      GoalHandleProgress::SharedPtr,
      const std::shared_ptr<const CableProgressAction::Feedback> feedback,
      size_t cable_id);

  void cable_progress_result_callback(
      const GoalHandleProgress::WrappedResult &result, size_t cable_id);

  // Interlace
  void cables_interlace_goal_response_callback(
      const GoalHandleInterlace::SharedPtr &goal_handle, size_t cable_id1,
      size_t cable_id2);

  void cables_interlace_feedback_callback(
      GoalHandleInterlace::SharedPtr,
      const std::shared_ptr<const CablesInterlaceAction::Feedback> feedback,
      size_t cable_id1, size_t cable_id2);

  void cables_interlace_result_callback(
      const GoalHandleInterlace::WrappedResult &result, size_t cable_id1,
      size_t cable_id2);
};
} // namespace cable_plan_executor

#endif