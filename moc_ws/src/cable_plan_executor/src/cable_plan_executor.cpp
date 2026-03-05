// Standard libraries
#include <fstream>
#include <rclcpp/duration.hpp>

// Third-party
#include "nlohmann/json.hpp"

// Local libraries
#include "cable_plan_executor/cable_plan_executor.hpp"
#include "robot_utils/cable.hpp"
#include "robot_utils/geometry.hpp"
#include "robot_utils/log_utils.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace cable_plan_executor {

CablePlanExecutor::CablePlanExecutor() : LifecycleNode("cable_plan_executor") {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CablePlanExecutor::on_configure(
    const rclcpp_lifecycle::State &) { // Declare parameters
  skip_hitches_ = this->declare_parameter<bool>("skip_hitches", false);
  plan_filepath_ = this->declare_parameter<std::string>("plan_filepath", "");
  first_end_robot_names_ = this->declare_parameter<std::vector<std::string>>(
      "first_end_robot_names", std::vector<std::string>{});
  last_end_robot_names_ = this->declare_parameter<std::vector<std::string>>(
      "last_end_robot_names", std::vector<std::string>{});
  action_server_timeout_duration_ =
      this->declare_parameter<double>("action_server_timeout", 5.0);

  // Generate Cables vector
  if (first_end_robot_names_.size() != last_end_robot_names_.size()) {
    throw std::runtime_error(
        "Attempting to use " + std::to_string(first_end_robot_names_.size()) +
        " first end robots with " +
        std::to_string(last_end_robot_names_.size()) + " last end robots. ");
  }
  for (size_t i{0}; i < first_end_robot_names_.size(); ++i) {
    cables_.push_back(robot_utils::cable::Cable{"cable" + std::to_string(i),
                                                first_end_robot_names_[i],
                                                last_end_robot_names_[i]});
  }

  load_plan();
  if (cables_.size() != joint_plan_.size()) {
    throw std::runtime_error("Attempting to use " +
                             std::to_string(cables_.size()) +
                             " cables. But plan contains " +
                             std::to_string(joint_plan_.size()) + ". ");
  }

  // Cable Move Action
  move_action_client_ptr_ =
      rclcpp_action::create_client<CableMoveAction>(this, "cable_move_action");

  RCLCPP_INFO(this->get_logger(), "Waiting for cable actions server...");
  std::chrono::duration<double> timeout_duration(
      action_server_timeout_duration_);

  if (!this->move_action_client_ptr_->wait_for_action_server(
          timeout_duration)) {
    throw std::runtime_error(
        "Cable move action server not available after wait");
  }

  // Cables Hitch Action
  if (!skip_hitches_) {
    hitch_action_client_ptr_ = rclcpp_action::create_client<CablesHitchAction>(
        this, "cables_hitch_action");
    if (!this->hitch_action_client_ptr_->wait_for_action_server(
            timeout_duration)) {
      throw std::runtime_error(
          "Cables hitch action server not available after wait");
    }
  }

  RCLCPP_INFO(this->get_logger(), "Action server(s) found. Executing plan...");
  return CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CablePlanExecutor::on_activate(const rclcpp_lifecycle::State &) {
  is_active_ = true; // Enable sending goals again

  for (size_t i{0}; i < cables_.size(); ++i) {
    if (current_steps_[i] < joint_plan_[i].size()) {
      send_next_goal(i);
    } else {
      RCLCPP_INFO(this->get_logger(), "%s Does not have any actions to perform",
                  robot_utils::log::get_cable_prefix(cables_[i]).c_str());
    }
  }
  return CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CablePlanExecutor::on_deactivate(const rclcpp_lifecycle::State &) {
  is_active_ = false;
  // Implement any cleanup needed during deactivate
  RCLCPP_INFO(this->get_logger(), "Deactivating CablePlanExecutor...");
  // Example: cancel goals, stop timers, reset states if needed
  // TODO: Implement cancellation of goals if your application requires

  // Cancel all ongoing goals
  if (move_action_client_ptr_) {
    move_action_client_ptr_->async_cancel_all_goals();
  }
  if (hitch_action_client_ptr_) {
    hitch_action_client_ptr_->async_cancel_all_goals();
  }
  return CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CablePlanExecutor::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up CablePlanExecutor...");

  move_action_client_ptr_.reset();
  hitch_action_client_ptr_.reset();
  cables_.clear();
  joint_plan_.clear();
  current_steps_.clear();

  return CallbackReturn::SUCCESS;
}

void CablePlanExecutor::load_plan() {
  RCLCPP_INFO(this->get_logger(), "Loading plan from: %s",
              plan_filepath_.c_str());
  std::ifstream ifs(plan_filepath_);
  if (!ifs)
    throw std::runtime_error("Cannot open file: " + plan_filepath_);
  nlohmann::json j;
  ifs >> j;

  for (const auto &plan : j.at("plans")) {
    std::vector<robot_utils::cable::CableAction> cable_plan;
    for (size_t i{0}; i < plan.size(); ++i) {
      auto act = plan[i];
      bool add_act{true};
      std::string act_type_str = act["type"].get<std::string>();
      char act_type_char = act_type_str[0];
      robot_utils::cable::ActionType act_type =
          robot_utils::cable::action_type_from_char(act_type_char);
      robot_utils::cable::CableEnd end;
      std::vector<robot_utils::geometry::Coords<double>> points;
      size_t vertex_id{0};
      robot_utils::geometry::Coords<double> vertex_coord;
      switch (act_type) {
      case robot_utils::cable::ActionType::Move:
        end = robot_utils::cable::CableEnd::Last;
        // Parse waypoints
        for (const auto &wp : act["waypoints"]) {
          double x = wp["x"].get<double>();
          double y = wp["y"].get<double>();
          robot_utils::geometry::Coords<double> pt{x, y};
          points.push_back(pt);
        }
        break;
      case robot_utils::cable::ActionType::Hitch:
        if (skip_hitches_) {
          add_act = false;
          break;
        }
        end = static_cast<robot_utils::cable::CableEnd>(act["end"]);
        vertex_id = act["vertex_id"];
        hitch_coop_flags_.resize(vertex_id + 1, -1);
        // Parse exit point
        const auto ep = act["exit_point"];
        double exit_x = ep["x"].get<double>();
        double exit_y = ep["y"].get<double>();
        robot_utils::geometry::Coords<double> exit_pt{exit_x, exit_y};
        points.push_back(exit_pt);
        const auto vp = act["vertex_coords"];
        double vp_x = vp["x"].get<double>();
        double vp_y = vp["y"].get<double>();
        robot_utils::geometry::Coords<double> vertex_pt{vp_x, vp_y};
        vertex_coord = vertex_pt;
        break;
      }
      if (!add_act) {
        continue;
      }
      bool is_last_action =
          (end == robot_utils::cable::CableEnd::First ||
           i == plan.size() -
                    1); // First end hitches should always move to the exit,
                        // last end hitches only if they are the last act
      robot_utils::cable::CableAction action{
          act_type, end, points, vertex_id, vertex_coord, is_last_action};
      cable_plan.push_back(action);
    }
    joint_plan_.push_back(cable_plan);
    current_steps_.push_back(0);
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu cable plans.",
              joint_plan_.size());
}

CablePlanExecutor::CableMoveAction::Goal
CablePlanExecutor::create_cable_move_action_goal(
    const robot_utils::cable::CableAction &move_action, const size_t cable_id) {
  CableMoveAction::Goal goal_msg;
  goal_msg.cable_id = cable_id;
  for (const auto &pt : move_action.points) {
    geometry_msgs::msg::Point point_msg;
    point_msg.x = pt.x;
    point_msg.y = pt.y;
    goal_msg.waypoints.push_back(point_msg);
  }
  return goal_msg;
}

CablePlanExecutor::CablesHitchAction::Goal
CablePlanExecutor::create_cables_hitch_action_goal(
    const robot_utils::cable::CableAction &hitch_action1,
    const size_t cable_id1,
    const robot_utils::cable::CableAction &hitch_action2,
    const size_t cable_id2) {
  CablesHitchAction::Goal goal_msg;
  goal_msg.cable_id1 = cable_id1;
  goal_msg.cable_id2 = cable_id2;
  geometry_msgs::msg::Point exit_point_msg1;
  exit_point_msg1.x = hitch_action1.points[0].x;
  exit_point_msg1.y = hitch_action1.points[0].y;
  goal_msg.exit_point1 = exit_point_msg1;
  geometry_msgs::msg::Point exit_point_msg2;
  exit_point_msg2.x = hitch_action2.points[0].x;
  exit_point_msg2.y = hitch_action2.points[0].y;
  goal_msg.exit_point2 = exit_point_msg2;
  goal_msg.cable_end1 = static_cast<int>(hitch_action1.end);
  goal_msg.cable_end2 = static_cast<int>(hitch_action2.end);
  goal_msg.is_1last = hitch_action1.is_last;
  goal_msg.is_2last = hitch_action2.is_last;
  geometry_msgs::msg::Point vertex_coord_msg;
  vertex_coord_msg.x = hitch_action1.vertex_coords.x;
  vertex_coord_msg.y = hitch_action1.vertex_coords.y;
  goal_msg.vertex_coords = vertex_coord_msg;
  return goal_msg;
}

void CablePlanExecutor::send_next_goal(size_t cable_id) {
  if (!is_active_) {
    return;
  }
  size_t current_act_idx_on_plan = current_steps_[cable_id];
  robot_utils::cable::CableAction action =
      joint_plan_[cable_id][current_act_idx_on_plan];

  switch (action.type) {
  case robot_utils::cable::ActionType::Move: {
    auto goal_msg = create_cable_move_action_goal(action, cable_id);
    RCLCPP_INFO(
        this->get_logger(),
        "%s %s Sending action request (%zu/%zu) - cable move action",
        robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str(),
        robot_utils::log::get_robot_prefix(last_end_robot_names_[cable_id])
            .c_str(),
        current_act_idx_on_plan, joint_plan_[cable_id].size());
    auto send_goal_options =
        rclcpp_action::Client<CableMoveAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&CablePlanExecutor::cable_move_goal_response_callback, this,
                  _1, cable_id);
    send_goal_options.feedback_callback =
        std::bind(&CablePlanExecutor::cable_move_feedback_callback, this, _1,
                  _2, cable_id);
    send_goal_options.result_callback = std::bind(
        &CablePlanExecutor::cable_move_result_callback, this, _1, cable_id);
    this->move_action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    break;
  }
  case robot_utils::cable::ActionType::Hitch:
    // Before the hitch action request can be made we need to check for
    // cooperation
    RCLCPP_INFO(
        this->get_logger(), "%s %s Ready to hitch",
        robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str(),
        robot_utils::log::get_robot_prefix(
            robot_utils::cable::get_robot_at_end(cables_[cable_id], action.end))
            .c_str());
    int coop_cable_id = hitch_coop_flags_[action.vertex_id];
    if (coop_cable_id >= 0) { // another robot is ready to coop
      size_t coop_cable_hitch_act_idx = current_steps_[coop_cable_id];
      robot_utils::cable::CableAction coop_robot_hitch_action =
          joint_plan_[coop_cable_id][coop_cable_hitch_act_idx];
      auto goal_msg = create_cables_hitch_action_goal(
          action, cable_id, coop_robot_hitch_action, coop_cable_id);
      RCLCPP_INFO(
          this->get_logger(),
          "%s %s Sending action request (%zu/%zu) - cables hitch action",
          robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str(),
          robot_utils::log::get_robot_prefix(
              robot_utils::cable::get_robot_at_end(cables_[cable_id],
                                                   action.end))
              .c_str(),
          current_act_idx_on_plan, joint_plan_[cable_id].size());
      RCLCPP_INFO(
          this->get_logger(),
          "%s %s Sending action request (%zu/%zu) - cables hitch action",
          robot_utils::log::get_cable_prefix(cables_[coop_cable_id]).c_str(),
          robot_utils::log::get_robot_prefix(
              robot_utils::cable::get_robot_at_end(cables_[cable_id],
                                                   coop_robot_hitch_action.end))
              .c_str(),
          coop_cable_hitch_act_idx, joint_plan_[coop_cable_id].size());
      auto send_goal_options =
          rclcpp_action::Client<CablesHitchAction>::SendGoalOptions();
      send_goal_options.goal_response_callback =
          std::bind(&CablePlanExecutor::cables_hitch_goal_response_callback,
                    this, _1, cable_id, coop_cable_id);
      send_goal_options.feedback_callback =
          std::bind(&CablePlanExecutor::cables_hitch_feedback_callback, this,
                    _1, _2, cable_id, coop_cable_id);
      send_goal_options.result_callback =
          std::bind(&CablePlanExecutor::cables_hitch_result_callback, this, _1,
                    cable_id, coop_cable_id);
      this->hitch_action_client_ptr_->async_send_goal(goal_msg,
                                                      send_goal_options);
    } else {
      hitch_coop_flags_[action.vertex_id] = cable_id;
    }
    break;
  }
}

void CablePlanExecutor::cable_move_goal_response_callback(
    const GoalHandleMove::SharedPtr &goal_handle, size_t cable_id) {
  if (!goal_handle) {
    RCLCPP_ERROR(
        this->get_logger(), "%s %s Cable Move Goal was rejected by server",
        robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str(),
        robot_utils::log::get_robot_prefix(last_end_robot_names_[cable_id])
            .c_str());
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "%s %s Cable Move Goal accepted by server, waiting for result",
        robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str(),
        robot_utils::log::get_robot_prefix(last_end_robot_names_[cable_id])
            .c_str());
  }
}

void CablePlanExecutor::cable_move_feedback_callback(
    GoalHandleMove::SharedPtr,
    const std::shared_ptr<const CableMoveAction::Feedback> feedback,
    size_t cable_id) {
  RCLCPP_INFO(
      this->get_logger(), "%s %s Feedback received. Remaining waypoints: %d",
      robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str(),
      robot_utils::log::get_robot_prefix(last_end_robot_names_[cable_id])
          .c_str(),
      feedback->remaining_waypoints);
}

void CablePlanExecutor::cable_move_result_callback(
    const GoalHandleMove::WrappedResult &result, size_t cable_id) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(
        this->get_logger(), "%s %s Cable Move Goal was aborted",
        robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str(),
        robot_utils::log::get_robot_prefix(last_end_robot_names_[cable_id])
            .c_str());
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(
        this->get_logger(), "%s %s Cable Move Goal was canceled",
        robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str(),
        robot_utils::log::get_robot_prefix(last_end_robot_names_[cable_id])
            .c_str());
    return;
  default:
    RCLCPP_ERROR(
        this->get_logger(), "%s %s Unknown result code",
        robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str(),
        robot_utils::log::get_robot_prefix(last_end_robot_names_[cable_id])
            .c_str());
    return;
  }
  RCLCPP_INFO(
      this->get_logger(), "%s %s Cable Move Goal reached.",
      robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str(),
      robot_utils::log::get_robot_prefix(last_end_robot_names_[cable_id])
          .c_str());
  current_steps_[cable_id]++;
  if (current_steps_[cable_id] < joint_plan_[cable_id].size() && is_active_) {
    send_next_goal(cable_id);
  } else {
    RCLCPP_INFO(this->get_logger(), "%s Has completed all of its actions",
                robot_utils::log::get_cable_prefix(cables_[cable_id]).c_str());
  }
}

void CablePlanExecutor::cables_hitch_goal_response_callback(
    const GoalHandleHitch::SharedPtr &goal_handle, size_t cable_id1,
    size_t cable_id2) {
  if (!goal_handle) {
    RCLCPP_ERROR(
        this->get_logger(), "%s-%s Cables Hitch Goal was rejected by server",
        robot_utils::log::get_cable_prefix(cables_[cable_id1]).c_str(),
        robot_utils::log::get_cable_prefix(cables_[cable_id2]).c_str());
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "%s-%s Cables Hitch Goal accepted by server, waiting for result",
        robot_utils::log::get_cable_prefix(cables_[cable_id1]).c_str(),
        robot_utils::log::get_cable_prefix(cables_[cable_id2]).c_str());
  }
}

void CablePlanExecutor::cables_hitch_feedback_callback(
    GoalHandleHitch::SharedPtr,
    const std::shared_ptr<const CablesHitchAction::Feedback> feedback,
    size_t cable_id1, size_t cable_id2) {
  RCLCPP_INFO(this->get_logger(),
              "%s-%s Feedback received. Current hitch step: %d",
              robot_utils::log::get_cable_prefix(cables_[cable_id1]).c_str(),
              robot_utils::log::get_cable_prefix(cables_[cable_id2]).c_str(),
              feedback->current_hitch_step);
}

void CablePlanExecutor::cables_hitch_result_callback(
    const GoalHandleHitch::WrappedResult &result, size_t cable_id1,
    size_t cable_id2) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(
        this->get_logger(), "%s-%s Cables Hitch Goal was aborted",
        robot_utils::log::get_cable_prefix(cables_[cable_id1]).c_str(),
        robot_utils::log::get_cable_prefix(cables_[cable_id2]).c_str());
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(
        this->get_logger(), "%s-%s Cables Hitch Goal was canceled",
        robot_utils::log::get_cable_prefix(cables_[cable_id1]).c_str(),
        robot_utils::log::get_cable_prefix(cables_[cable_id2]).c_str());
    return;
  default:
    RCLCPP_ERROR(
        this->get_logger(), "%s-%s Unknown result code",
        robot_utils::log::get_cable_prefix(cables_[cable_id1]).c_str(),
        robot_utils::log::get_cable_prefix(cables_[cable_id2]).c_str());
    return;
  };
  RCLCPP_INFO(this->get_logger(), "%s-%s Cables Hitch Goal reached.",
              robot_utils::log::get_cable_prefix(cables_[cable_id1]).c_str(),
              robot_utils::log::get_cable_prefix(cables_[cable_id2]).c_str());

  current_steps_[cable_id1]++;
  current_steps_[cable_id2]++;
  if (current_steps_[cable_id1] < joint_plan_[cable_id1].size() && is_active_) {
    send_next_goal(cable_id1);
  } else {
    RCLCPP_INFO(this->get_logger(), "%s Has completed all of its actions",
                robot_utils::log::get_cable_prefix(cables_[cable_id1]).c_str());
  }
  if (current_steps_[cable_id2] < joint_plan_[cable_id2].size() && is_active_) {
    send_next_goal(cable_id2);
  } else {
    RCLCPP_INFO(this->get_logger(), "%s Has completed all of its actions",
                robot_utils::log::get_cable_prefix(cables_[cable_id2]).c_str());
  }
}

} // namespace cable_plan_executor