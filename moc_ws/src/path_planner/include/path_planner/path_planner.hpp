#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

// Standard libraries

// ROS2 core
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Local libraries
#include "path_planner/image_map.hpp"
#include "robot_utils/tf_listener_wrapper.hpp"

// Messages
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

// Local messages
#include <path_planning_interfaces/action/path_plan.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <robot_utils/geometry.hpp>
#include <string>

namespace path_planner {

struct HPlanningNode {
  robot_utils::geometry::Coords<int> coords;
  std::vector<int> h_sig;

  bool is_coords_equal(const HPlanningNode &n) const {
    return coords == n.coords;
  }

  bool operator==(const HPlanningNode &n) const {
    if (!is_coords_equal(n)) {
      return false;
    }
    if (h_sig.size() != n.h_sig.size()) {
      return false;
    }
    for (size_t i{0}; i < h_sig.size(); ++i) {
      if (h_sig[i] != n.h_sig[i]) {
        return false;
      }
    }
    return true;
  }
};

struct PlanningGraph {
  ImageMap<int> map;

  std::vector<robot_utils::geometry::Coords<double>> other_robot_pos_collision;
  robot_utils::geometry::Pose2D other_robot_pose_h_sig;

  double robot_radius;
  double inter_robot_collision_check_factor;

  double robot_push_scale_default;
  double robot_push_scale_step;
  int robot_push_scale_max_attempts;
  double boundary_push_scale_default;
  double boundary_push_scale_step;
  int boundary_push_scale_max_attempts;
  double obstacle_push_scale_default;
  double obstacle_push_scale_step;
  int obstacle_push_scale_max_attempts;

  std::vector<int>
  get_robot_idx_in_collision(const robot_utils::geometry::Coords<int> &n) const;
  bool in_collision_with_obstacles(
      const robot_utils::geometry::Coords<int> &n) const;
  bool within_map_boundaries(const robot_utils::geometry::Coords<int> &n) const;
  robot_utils::geometry::Coords<int>
  push_inside_boundaries(const robot_utils::geometry::Coords<int> &n,
                         double d) const;
  robot_utils::geometry::Coords<int>
  get_valid_coord(const robot_utils::geometry::Coords<int> &n) const;

  bool
  edge_crosses_h_ray(const robot_utils::geometry::Coords<int> &n1,
                     const robot_utils::geometry::Coords<int> &n2,
                     const robot_utils::geometry::Coords<double> &rep_pt,
                     const robot_utils::geometry::Coords<double> &rep_heading,
                     int &cross_dir) const;
  void update_h_sig(const HPlanningNode &n, HPlanningNode &tn) const;
  std::vector<std::pair<HPlanningNode, double>>
  neighbors(const HPlanningNode &n) const;
  std::vector<std::pair<robot_utils::geometry::Coords<int>, double>>
  neighbors(const robot_utils::geometry::Coords<int> &n) const;
};

class PathPlanner : public rclcpp::Node {
public:
  using PathPlanningAction = path_planning_interfaces::action::PathPlan;
  using PathPlanningGoalHandle =
      rclcpp_action::ServerGoalHandle<PathPlanningAction>;

  PathPlanner();
  ~PathPlanner();
  void initialize();
  int get_n_threads() const;

private:
  // Parameters
  int n_threads_;
  double robot_radius_;
  double inter_robot_collision_check_factor_;
  std::string map_yaml_filepath_;
  std::vector<std::string> robot_names_;
  std::string target_frame_;
  double tf_timeout_duration_;
  double image_coords_scale_;
  double robot_push_scale_default_;
  double robot_push_scale_step_;
  int robot_push_scale_max_attempts_;
  double boundary_push_scale_default_;
  double boundary_push_scale_step_;
  int boundary_push_scale_max_attempts_;
  double obstacle_push_scale_default_;
  double obstacle_push_scale_step_;
  int obstacle_push_scale_max_attempts_;

  std::map<rclcpp_action::GoalUUID, std::thread> active_threads_;
  std::mutex thread_mutex_;

  // tf
  std::unique_ptr<robot_utils::tf::TfListenerWrapper> tf_wrapper_;

  // Action Server
  rclcpp_action::Server<PathPlanningAction>::SharedPtr
      path_planning_server_ptr_;

  // Publisher
  std::unordered_map<std::string,
                     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr>
      robot_path_pub_ptrs_;
  std::unordered_map<
      std::string,
      rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr>
      robot_path_goal_pub_ptrs_;

  // Planning variables
  std::unordered_map<std::string, PlanningGraph> planning_graphs_;
  PlanningGraph base_planning_graph_;

  // Action Handlers
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const PathPlanningAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<PathPlanningGoalHandle> goal_handle);
  void
  handle_accepted(const std::shared_ptr<PathPlanningGoalHandle> goal_handle);

  void execute(const std::shared_ptr<PathPlanningGoalHandle> goal_handle);

  bool
  prep_planning_problem(robot_utils::geometry::Coords<int> &start_coords,
                        robot_utils::geometry::Coords<int> &goal_coords,
                        const std::string &robot_name,
                        const robot_utils::geometry::Coords<double> goal_pos,
                        std::string &msg,
                        const std::string &other_robot_name = "");

  // Main logic
  bool plan_path(const std::string &robot_name,
                 const robot_utils::geometry::Coords<double> goal_pos,
                 std::vector<geometry_msgs::msg::Point> &path,
                 std::string &error_msg,
                 std::function<bool()> pretermination_func);
  bool plan_homotopy_path(const std::string &robot_name,
                          const robot_utils::geometry::Coords<double> goal_pos,
                          const std::string &other_robot_name,
                          const int &h_sig_goal_robot,
                          std::vector<geometry_msgs::msg::Point> &path,
                          std::string &error_msg,
                          std::function<bool()> pretermination_func);

  robot_utils::geometry::Pose2D
  transform_real_to_image(const robot_utils::geometry::Pose2D &pose) const;

  template <typename OutCoordType>
  robot_utils::geometry::Coords<OutCoordType> transform_real_to_image(
      const robot_utils::geometry::Coords<double> &pos) const;

  robot_utils::geometry::Coords<double>
  transform_image_to_real(const robot_utils::geometry::Coords<int> &pos) const;
};
} // namespace path_planner

#endif