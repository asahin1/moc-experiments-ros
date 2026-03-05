// Standard libraries

#include <cstddef>
#include <exception>
#include <functional>

// ROS2 Core

// Third-party
#include <custom_mpl/search/algorithms/astar.hpp>
#include <custom_mpl/search/policies/closed_set.hpp>
#include <custom_mpl/search/policies/reopen.hpp>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/create_server.hpp>
#include <robot_utils/geometry.hpp>
#include <std_msgs/msg/detail/int16__struct.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

// Messages

// Local libraries
#include "path_planner/path_planner.hpp"
#include "robot_utils/log_utils.hpp"

// Local messages

using namespace std::chrono_literals;
using namespace std::placeholders;

template <> struct std::hash<path_planner::HPlanningNode> {
  std::size_t operator()(const path_planner::HPlanningNode &s) const noexcept {
    return std::hash<robot_utils::geometry::Coords<int>>{}(s.coords);
  }
};

namespace path_planner {

std::vector<int> PlanningGraph::get_robot_idx_in_collision(
    const robot_utils::geometry::Coords<int> &n) const {
  std::vector<int> res;
  for (size_t i{0}; i < other_robot_pos_collision.size(); ++i) {
    double dist = robot_utils::geometry::euclideanDistance2D(
        n, other_robot_pos_collision[i]);
    if (dist < robot_radius * inter_robot_collision_check_factor) {
      res.push_back(i);
    }
  }
  return res;
}

bool PlanningGraph::in_collision_with_obstacles(
    const robot_utils::geometry::Coords<int> &n) const {
  return !map.isObstacleFree(n);
}

bool PlanningGraph::within_map_boundaries(
    const robot_utils::geometry::Coords<int> &n) const {
  return map.inWorkspace(n);
}

robot_utils::geometry::Coords<int> PlanningGraph::push_inside_boundaries(
    const robot_utils::geometry::Coords<int> &n, double d) const {
  return map.push_inside_boundary(n, d);
}

robot_utils::geometry::Coords<int> PlanningGraph::get_valid_coord(
    const robot_utils::geometry::Coords<int> &n) const {
  robot_utils::geometry::Coords<int> valid_n = n;
  // Handle collision with other robots
  auto other_robots_in_collision = get_robot_idx_in_collision(valid_n);
  int push_attempts{0};
  while (!other_robots_in_collision.empty()) {
    for (size_t i{0}; i < other_robots_in_collision.size(); ++i) {
      size_t collision_idx = other_robots_in_collision[i];
      // Apply push away
      robot_utils::geometry::Coords<double> v =
          (valid_n - other_robot_pos_collision[collision_idx]).normalized();
      auto new_n = valid_n + v * robot_radius *
                                 (robot_push_scale_default +
                                  push_attempts * robot_push_scale_step);
      valid_n.x = new_n.x;
      valid_n.y = new_n.y;
    }
    other_robots_in_collision = get_robot_idx_in_collision(valid_n);
    push_attempts++;
    if (push_attempts == robot_push_scale_max_attempts) {
      throw std::runtime_error("Cannot find a valid planning point that is "
                               "without robot collision in max pushes.");
    }
  }
  // Handle boundary violations
  push_attempts = 0;
  while (!within_map_boundaries(valid_n)) {
    valid_n = push_inside_boundaries(
        valid_n, robot_radius * (boundary_push_scale_default +
                                 push_attempts * boundary_push_scale_step));
    other_robots_in_collision = get_robot_idx_in_collision(valid_n);
    if (!other_robots_in_collision.empty()) {
      throw std::runtime_error(
          "Cannot find a valid planning point that is "
          "within boundaries and without robot collision.");
    }
    push_attempts++;
    if (push_attempts == boundary_push_scale_max_attempts) {
      throw std::runtime_error("Cannot find a valid planning point that is "
                               "within map boundaries in max pushes.");
    }
  }
  // Handle obstacle collisions
  if (!in_collision_with_obstacles(valid_n)) {
    return valid_n;
  }
  auto neighbors = map.allNeighbors(n);
  for (int p{0}; p < obstacle_push_scale_max_attempts; ++p) {
    for (const auto &neighbor : neighbors) {
      // Apply pull towards neighbors
      robot_utils::geometry::Coords<double> v =
          (valid_n - neighbor.first).normalized();
      auto new_n = valid_n + v * robot_radius *
                                 (obstacle_push_scale_default +
                                  p * obstacle_push_scale_step);
      robot_utils::geometry::Coords<int> int_coords;
      int_coords.x = new_n.x;
      int_coords.y = new_n.y;
      if (get_robot_idx_in_collision(int_coords).empty() &&
          within_map_boundaries(int_coords)) {
        valid_n = int_coords;
        return valid_n;
      }
    }
  }
  throw std::runtime_error("Cannot find a valid planning point that is "
                           "without obstacle collision.");
}

bool PlanningGraph::edge_crosses_h_ray(
    const robot_utils::geometry::Coords<int> &n1,
    const robot_utils::geometry::Coords<int> &n2,
    const robot_utils::geometry::Coords<double> &rep_pt,
    const robot_utils::geometry::Coords<double> &rep_heading,
    int &cross_dir) const {
  const double EPS = 1e-9;
  robot_utils::geometry::Coords<double> r = n2 - n1;
  robot_utils::geometry::Coords<double> v = rep_pt - n1;
  double denom = robot_utils::geometry::cross(r, rep_heading);
  if (std::abs(denom) < EPS) {
    return false;
  }
  double u = robot_utils::geometry::cross(v, rep_heading) / denom;
  double t = robot_utils::geometry::cross(v, r) / denom;

  if (u >= 0.0 && u <= 1.0 && t >= 0.0) {
    cross_dir = (denom > 0.0) ? +1 : -1;
    return true;
  }
  return false;
}

void PlanningGraph::update_h_sig(const HPlanningNode &n,
                                 HPlanningNode &tn) const {
  tn.h_sig = n.h_sig;
  for (size_t i{0}; i < map.parsed_map.repPts.size(); ++i) {
    robot_utils::geometry::Coords<double> obj_rep_pt;
    obj_rep_pt.x = map.parsed_map.repPts[i].x;
    obj_rep_pt.y = map.parsed_map.repPts[i].y;
    robot_utils::geometry::Coords<double> obj_ray_heading{0.0, -1.0};
    int dir{0};
    if (edge_crosses_h_ray(n.coords, tn.coords, obj_rep_pt, obj_ray_heading,
                           dir)) {
      tn.h_sig[i] += dir;
    }
  }
  int dir{0};
  if (edge_crosses_h_ray(n.coords, tn.coords, other_robot_pose_h_sig.position,
                         other_robot_pose_h_sig.heading, dir)) {
    tn.h_sig.back() += dir;
  }
}

std::vector<std::pair<HPlanningNode, double>>
PlanningGraph::neighbors(const HPlanningNode &n) const {

  auto neighbors = map.validNeighbors(n.coords);
  std::vector<std::pair<HPlanningNode, double>> res;
  for (const auto &neighbor : neighbors) {
    if (!get_robot_idx_in_collision(neighbor.first).empty()) {
      continue;
    }
    HPlanningNode hp;
    hp.coords = neighbor.first;
    update_h_sig(n, hp);
    res.push_back({hp, neighbor.second});
  }
  return res;
}

std::vector<std::pair<robot_utils::geometry::Coords<int>, double>>
PlanningGraph::neighbors(const robot_utils::geometry::Coords<int> &n) const {

  auto neighbors = map.validNeighbors(n);
  std::vector<std::pair<robot_utils::geometry::Coords<int>, double>> res;
  for (const auto &neighbor : neighbors) {
    if (!get_robot_idx_in_collision(neighbor.first).empty()) {
      continue;
    }
    res.push_back({neighbor.first, neighbor.second});
  }
  return res;
}

PathPlanner::PathPlanner() : Node("path_planner") {
  n_threads_ = this->declare_parameter<int>("n_threads", 4);
  robot_names_ = this->declare_parameter<std::vector<std::string>>(
      "robot_names", std::vector<std::string>{});
  robot_radius_ = this->declare_parameter<double>("robot_radius", 0.15);
  inter_robot_collision_check_factor_ = this->declare_parameter<double>(
      "inter_robot_collision_check_factor", 2.5);
  if (inter_robot_collision_check_factor_ < 2.0) {
    RCLCPP_WARN(this->get_logger(),
                "Path planner will use an inter robot collision check factor "
                "less than 2. This may result in inter robot collisions.");
  }
  target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
  tf_timeout_duration_ = this->declare_parameter<double>("tf_timeout", 5.0);
  image_coords_scale_ =
      this->declare_parameter<double>("image_coords_scale", 100.0);

  robot_push_scale_default_ =
      this->declare_parameter<double>("robot_push_scale_default", 5.0);
  robot_push_scale_step_ =
      this->declare_parameter<double>("robot_push_scale_step", 5.0);
  robot_push_scale_max_attempts_ =
      this->declare_parameter<int>("robot_push_scale_max_attempts", 5);
  boundary_push_scale_default_ =
      this->declare_parameter<double>("boundary_push_scale_default", 5.0);
  boundary_push_scale_step_ =
      this->declare_parameter<double>("boundary_push_scale_step", 5.0);
  boundary_push_scale_max_attempts_ =
      this->declare_parameter<int>("boundary_push_scale_max_attempts", 5);
  obstacle_push_scale_default_ =
      this->declare_parameter<double>("obstacle_push_scale_default", 5.0);
  obstacle_push_scale_step_ =
      this->declare_parameter<double>("obstacle_push_scale_step", 5.0);
  obstacle_push_scale_max_attempts_ =
      this->declare_parameter<int>("obstacle_push_scale_max_attempts_", 5);

  // Get map file and generate static map (with added inflation)
  map_yaml_filepath_ =
      this->declare_parameter<std::string>("map_yaml_filepath", "");
  RCLCPP_INFO(this->get_logger(), "Loading map yaml from: %s",
              map_yaml_filepath_.c_str());
  YAML::Node map_yaml = YAML::LoadFile(map_yaml_filepath_);
  std::string map_img_filepath =
      "resources/maps/" + map_yaml["image"].as<std::string>();
  RCLCPP_INFO(this->get_logger(), "Loading map image from: %s",
              map_img_filepath.c_str());

  planning_graph_.robot_radius = robot_radius_ * image_coords_scale_;
  planning_graph_.inter_robot_collision_check_factor =
      inter_robot_collision_check_factor_;
  ImageMap<int> searchMap;
  searchMap.loadMap(map_img_filepath, true,
                    planning_graph_.robot_radius); // inflate via Minkowski sum
  // searchMap.initializePlot(1);
  // searchMap.renderDisplay(0);
  planning_graph_.map = searchMap;
  planning_graph_.robot_push_scale_default = robot_push_scale_default_;
  planning_graph_.robot_push_scale_step = robot_push_scale_step_;
  planning_graph_.robot_push_scale_max_attempts =
      robot_push_scale_max_attempts_;
  planning_graph_.boundary_push_scale_default = boundary_push_scale_default_;
  planning_graph_.boundary_push_scale_step = boundary_push_scale_step_;
  planning_graph_.boundary_push_scale_max_attempts =
      boundary_push_scale_max_attempts_;
  planning_graph_.obstacle_push_scale_default = obstacle_push_scale_default_;
  planning_graph_.obstacle_push_scale_step = obstacle_push_scale_step_;
  planning_graph_.obstacle_push_scale_max_attempts =
      obstacle_push_scale_max_attempts_;

  RCLCPP_INFO(this->get_logger(),
              "Planning graph - robot_push_scale_default: %f",
              planning_graph_.robot_push_scale_default);

  RCLCPP_INFO(this->get_logger(), "Planning graph - robot_push_scale_step: %f",
              planning_graph_.robot_push_scale_step);

  RCLCPP_INFO(this->get_logger(),
              "Planning graph - robot_push_scale_max_attempts: %d",
              planning_graph_.robot_push_scale_max_attempts);

  RCLCPP_INFO(this->get_logger(),
              "Planning graph - boundary_push_scale_default: %f",
              planning_graph_.boundary_push_scale_default);

  RCLCPP_INFO(this->get_logger(),
              "Planning graph - boundary_push_scale_step: %f",
              planning_graph_.boundary_push_scale_step);

  RCLCPP_INFO(this->get_logger(),
              "Planning graph - boundary_push_scale_max_attempts: %d",
              planning_graph_.boundary_push_scale_max_attempts);

  for (size_t i{0}; i < robot_names_.size(); ++i) {
    robot_path_pub_ptrs_.insert(
        {robot_names_[i], this->create_publisher<nav_msgs::msg::Path>(
                              robot_names_[i] + "/planned_path", 10)});
    robot_path_goal_pub_ptrs_.insert(
        {robot_names_[i],
         this->create_publisher<geometry_msgs::msg::PointStamped>(
             robot_names_[i] + "/path_goal", 10)});
  }

  path_planning_server_ptr_ = rclcpp_action::create_server<PathPlanningAction>(
      this, "plan_path", std::bind(&PathPlanner::handle_goal, this, _1, _2),
      std::bind(&PathPlanner::handle_cancel, this, _1),
      std::bind(&PathPlanner::handle_accepted, this, _1));
}

PathPlanner::~PathPlanner() {
  std::lock_guard<std::mutex> lock(thread_mutex_);
  for (auto &[uuid, thread] : active_threads_) {
    if (thread.joinable()) {
      thread.join(); // Block until the thread finishes its current loop
    }
  }
}

void PathPlanner::initialize() {
  RCLCPP_DEBUG(this->get_logger(), "Creating tf wrapper.");
  tf_wrapper_ = std::make_unique<robot_utils::tf::TfListenerWrapper>(
      this->shared_from_this());
  RCLCPP_DEBUG(this->get_logger(), "tf wrapper created.");

  RCLCPP_INFO(this->get_logger(), "Path Planning server initialized.");
}

int PathPlanner::get_n_threads() const { return n_threads_; }

rclcpp_action::GoalResponse
PathPlanner::handle_goal(const rclcpp_action::GoalUUID &uuid,
                         std::shared_ptr<const PathPlanningAction::Goal> goal) {
  std::string robot_name = goal->robot_name;
  geometry_msgs::msg::Point goal_pt = goal->goal;
  std::string other_robot_name = goal->other_robot_name;
  RCLCPP_INFO(this->get_logger(),
              "%s Received path planning request to target: (%f, %f) - with "
              "other robot: %s",
              robot_utils::log::get_robot_prefix(robot_name).c_str(), goal_pt.x,
              goal_pt.y, other_robot_name.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathPlanner::handle_cancel(
    const std::shared_ptr<PathPlanningGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(),
              "Received request to cancel path plan action");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PathPlanner::handle_accepted(
    const std::shared_ptr<PathPlanningGoalHandle> goal_handle) {
  std::lock_guard<std::mutex> lock(thread_mutex_);
  const auto goal_id = goal_handle->get_goal_id();
  RCLCPP_INFO(this->get_logger(), "Executing path plan action");
  active_threads_[goal_id] =
      std::thread(std::bind(&PathPlanner::execute, this, std::placeholders::_1),
                  goal_handle);
}

void PathPlanner::execute(
    const std::shared_ptr<PathPlanningGoalHandle> goal_handle) {
  // Read action request
  const auto request = goal_handle->get_goal();
  std::string robot_name = request->robot_name;
  geometry_msgs::msg::Point goal_pt = request->goal;
  std::string other_robot_name = request->other_robot_name;
  int h_sig = request->h_sig;

  auto feedback = std::make_shared<PathPlanningAction::Feedback>();
  auto result = std::make_shared<PathPlanningAction::Result>();

  // Publish planning goal (for rviz)
  geometry_msgs::msg::PointStamped goal_stamped;
  goal_stamped.header.frame_id = target_frame_;
  goal_stamped.point = goal_pt;
  robot_path_goal_pub_ptrs_[robot_name]->publish(goal_stamped);

  // Run path planning algorithm
  robot_utils::geometry::Coords<double> goal{goal_pt.x, goal_pt.y};
  std::vector<geometry_msgs::msg::Point> path;
  std::string msg;
  auto should_cancel = [&]() { return goal_handle->is_canceling(); };
  bool plan_success = false;
  if (other_robot_name.empty()) {
    plan_success = plan_path(robot_name, goal, path, msg, should_cancel);
  } else {
    plan_success = plan_homotopy_path(robot_name, goal, other_robot_name, h_sig,
                                      path, msg, should_cancel);
  }
  if (goal_handle->is_canceling()) {
    // Case A: The library stopped because the ACTION was canceled
    RCLCPP_INFO(this->get_logger(), "Goal Canceled by Client");
    result->success = false;
    result->message = "USER_CANCELED";
    goal_handle->canceled(result);
    return;
  }

  // Generate action result
  if (!plan_success) {
    RCLCPP_WARN(this->get_logger(), "%s %s",
                robot_utils::log::get_robot_prefix(robot_name).c_str(),
                msg.c_str());
    result->success = false;
    result->message = msg;
    goal_handle->abort(result);
    return;
  }
  result->success = true;
  result->path = path;
  RCLCPP_DEBUG(this->get_logger(),
               "%s Sending planned path with %zu path points",
               robot_utils::log::get_robot_prefix(robot_name).c_str(),
               result->path.size());

  // Publish planned path (for rviz)
  nav_msgs::msg::Path planned_path_msg;
  planned_path_msg.header.frame_id = target_frame_;
  for (auto &p : path) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = target_frame_;
    pose_msg.pose.position.x = p.x;
    pose_msg.pose.position.y = p.y;
    planned_path_msg.poses.push_back(pose_msg);
  }
  robot_path_pub_ptrs_[robot_name]->publish(planned_path_msg);
  goal_handle->succeed(result);
}

bool PathPlanner::prep_planning_problem(
    robot_utils::geometry::Coords<int> &start_coords,
    robot_utils::geometry::Coords<int> &goal_coords,
    const std::string &robot_name,
    const robot_utils::geometry::Coords<double> goal_pos, std::string &msg,
    const std::string &other_robot_name) {

  rclcpp::Duration robot_pose_timeout =
      rclcpp::Duration::from_seconds(tf_timeout_duration_);

  // Get the pose of the planning robot from tf
  robot_utils::geometry::Coords<double> planning_robot_pos;
  bool planning_robot_pose_read = tf_wrapper_->lookup_2D_pos_with_timeout(
      target_frame_, robot_name, planning_robot_pos, robot_pose_timeout);
  if (!planning_robot_pose_read) {
    RCLCPP_ERROR(this->get_logger(), "%s Timeout while waiting for transform",
                 robot_utils::log::get_robot_prefix(robot_name).c_str());
    msg = "Failed to get planning robot transform.";
    return false;
  }

  // Get poses of other robots from tf and store them in the environment
  // object
  std::vector<robot_utils::geometry::Coords<double>> other_robot_positions;
  for (const auto &orn : robot_names_) {
    if (orn == robot_name) {
      continue;
    }
    if (orn == other_robot_name) {
      robot_utils::geometry::Pose2D other_robot_pose;
      bool other_robot_pose_read = tf_wrapper_->lookup_2D_pose_with_timeout(
          target_frame_, orn, other_robot_pose, robot_pose_timeout);
      if (!other_robot_pose_read) {
        RCLCPP_WARN(this->get_logger(),
                    "%s Timeout while waiting for other robot transform",
                    robot_utils::log::get_robot_prefix(robot_name).c_str());
        msg = "Failed to get other robot transform.";
        return false;
      }
      other_robot_positions.push_back(other_robot_pose.position);
      RCLCPP_DEBUG(this->get_logger(),
                   "%s Planning with other robot %s at (%f, %f) - real coords",
                   robot_utils::log::get_robot_prefix(robot_name).c_str(),
                   orn.c_str(), other_robot_pose.position.x,
                   other_robot_pose.position.y);
      planning_graph_.other_robot_pose_h_sig =
          transform_real_to_image(other_robot_pose);
    } else {
      robot_utils::geometry::Coords<double> other_robot_pos;
      bool other_robot_pose_read = tf_wrapper_->lookup_2D_pos_with_timeout(
          target_frame_, orn, other_robot_pos, robot_pose_timeout);
      if (!other_robot_pose_read) {
        RCLCPP_WARN(this->get_logger(),
                    "%s Timeout while waiting for other robot transform",
                    robot_utils::log::get_robot_prefix(robot_name).c_str());
        msg = "Failed to get other robot transform.";
        return false;
      }
      other_robot_positions.push_back(other_robot_pos);
      RCLCPP_DEBUG(this->get_logger(),
                   "%s Planning with other robot %s at (%f, %f) - real coords",
                   robot_utils::log::get_robot_prefix(robot_name).c_str(),
                   orn.c_str(), other_robot_pos.x, other_robot_pos.y);
    }
  }

  // Transform read real robot poses to image coords
  robot_utils::geometry::Coords<int> planning_robot_pose_transformed =
      transform_real_to_image<int>(planning_robot_pos);
  std::vector<robot_utils::geometry::Coords<double>>
      other_robot_poses_transformed;
  for (const auto &orp : other_robot_positions) {
    auto tp = transform_real_to_image<double>(orp);
    other_robot_poses_transformed.push_back(tp);
    RCLCPP_DEBUG(this->get_logger(),
                 "%s Planning with other robot at (%f, %f) - image coords",
                 robot_utils::log::get_robot_prefix(robot_name).c_str(), tp.x,
                 tp.y);
  }

  planning_graph_.other_robot_pos_collision = other_robot_poses_transformed;
  RCLCPP_DEBUG(this->get_logger(), "%s Planning with %zu other robots.",
               robot_utils::log::get_robot_prefix(robot_name).c_str(),
               planning_graph_.other_robot_pos_collision.size());

  robot_utils::geometry::Coords<int> start = planning_robot_pose_transformed;
  robot_utils::geometry::Coords<int> goal =
      transform_real_to_image<int>(goal_pos);

  // Start point is inside obstacle (or other robot) or out of image
  try {
    start_coords = planning_graph_.get_valid_coord(start);
    goal_coords = planning_graph_.get_valid_coord(goal);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(
        this->get_logger(),
        "%s Planning from (%d, %d) to (%d, %d) - image coords: not feasible",
        robot_utils::log::get_robot_prefix(robot_name).c_str(), start.x,
        start.y, goal.x, goal.y);
    msg = ex.what(); // Planning problem is not feasible.
    return false;
  }
  return true;
}

bool PathPlanner::plan_path(
    const std::string &robot_name,
    const robot_utils::geometry::Coords<double> goal_pos,
    std::vector<geometry_msgs::msg::Point> &path, std::string &msg,
    std::function<bool()> pretermination_func) {

  robot_utils::geometry::Coords<int> start;
  robot_utils::geometry::Coords<int> goal;
  bool prep_success =
      prep_planning_problem(start, goal, robot_name, goal_pos, msg);
  if (!prep_success) {
    return false;
  }

  auto is_goal = [&](const robot_utils::geometry::Coords<int> &n) {
    return n == goal;
  };

  auto euclidean_heuristic = [&](const robot_utils::geometry::Coords<int> &n) {
    return robot_utils::geometry::euclideanDistance2D(n, goal);
  };

  RCLCPP_DEBUG(this->get_logger(),
               "%s Planning from (%d, %d) to (%d, %d) - image coords",
               robot_utils::log::get_robot_prefix(robot_name).c_str(), start.x,
               start.y, goal.x, goal.y);

  auto [res, _] =
      custom_mpl::search::algorithms::astar<robot_utils::geometry::Coords<int>>(
          planning_graph_, start, is_goal, euclidean_heuristic,
          pretermination_func, custom_mpl::search::policies::ClosedNone{},
          custom_mpl::search::policies::ReopenForbid{});

  // Handle planning result - update path and msg, return success bool
  if (res.found) {
    if (res.path.empty()) {
      msg = "Empty path.";
      return false;
    } else {
      for (const auto &pt : res.path) {
        geometry_msgs::msg::Point pt_msg;
        robot_utils::geometry::Coords<double> real_pt =
            transform_image_to_real(pt);
        pt_msg.x = real_pt.x;
        pt_msg.y = real_pt.y;
        path.push_back(pt_msg);
      }
      msg = "Path found.";
      return true;
    }
  } else {
    msg = "Path planning algorithm could not find a solution.";
    return false;
  }
}

bool PathPlanner::plan_homotopy_path(
    const std::string &robot_name,
    const robot_utils::geometry::Coords<double> goal_pos,
    const std::string &other_robot_name, const int &h_sig_goal_robot,
    std::vector<geometry_msgs::msg::Point> &path, std::string &msg,
    std::function<bool()> pretermination_func) {

  robot_utils::geometry::Coords<int> start;
  robot_utils::geometry::Coords<int> goal;
  bool prep_success = prep_planning_problem(start, goal, robot_name, goal_pos,
                                            msg, other_robot_name);
  if (!prep_success) {
    return false;
  }

  HPlanningNode h_start;
  h_start.coords = start;
  h_start.h_sig.resize(planning_graph_.map.parsed_map.repPts.size() + 1, 0);
  HPlanningNode h_goal;
  h_goal.coords = goal;
  h_goal.h_sig = h_start.h_sig;
  h_goal.h_sig.back() = h_sig_goal_robot;

  auto is_goal = [&](const HPlanningNode &n) { return n == h_goal; };

  auto euclidean_heuristic = [&](const HPlanningNode &n) {
    return robot_utils::geometry::euclideanDistance2D(n.coords, goal);
  };

  RCLCPP_DEBUG(
      this->get_logger(),
      "%s H-Planning from (%d, %d, h=%d) to (%d, %d, h=%d) - image coords",
      robot_utils::log::get_robot_prefix(robot_name).c_str(), start.x, start.y,
      h_start.h_sig.back(), goal.x, goal.y, h_goal.h_sig.back());

  auto [res, _] = custom_mpl::search::algorithms::astar<HPlanningNode>(
      planning_graph_, h_start, is_goal, euclidean_heuristic,
      pretermination_func, custom_mpl::search::policies::ClosedNone{},
      custom_mpl::search::policies::ReopenForbid{});

  // Handle planning result - update path and msg, return success bool
  if (res.found) {
    if (res.path.empty()) {
      msg = "Empty path.";
      return false;
    } else {
      for (const auto &pt : res.path) {
        RCLCPP_DEBUG(this->get_logger(), "Path pt: %d, %d, h:%d", pt.coords.x,
                     pt.coords.y, pt.h_sig.back());
        geometry_msgs::msg::Point pt_msg;
        robot_utils::geometry::Coords<double> real_pt =
            transform_image_to_real(pt.coords);
        pt_msg.x = real_pt.x;
        pt_msg.y = real_pt.y;
        path.push_back(pt_msg);
      }
      msg = "Path found.";
      return true;
    }
  } else {
    msg = "Path planning algorithm could not find a solution.";
    return false;
  }
}

robot_utils::geometry::Pose2D PathPlanner::transform_real_to_image(
    const robot_utils::geometry::Pose2D &pose) const {
  robot_utils::geometry::Pose2D out;
  out.position.x = pose.position.x * image_coords_scale_;
  out.position.y = planning_graph_.map.parsed_map.height() -
                   pose.position.y * image_coords_scale_;
  out.heading.x = pose.heading.x;
  out.heading.y = -pose.heading.y;
  return out;
}

template <typename OutCoordType>
robot_utils::geometry::Coords<OutCoordType>
PathPlanner::transform_real_to_image(
    const robot_utils::geometry::Coords<double> &pos) const {
  robot_utils::geometry::Coords<OutCoordType> out;
  out.x = OutCoordType(pos.x * image_coords_scale_);
  out.y = planning_graph_.map.parsed_map.height() -
          OutCoordType(pos.y * image_coords_scale_);
  return out;
}

robot_utils::geometry::Coords<double> PathPlanner::transform_image_to_real(
    const robot_utils::geometry::Coords<int> &pos) const {
  robot_utils::geometry::Coords<double> out;
  out.x = static_cast<double>(pos.x) / image_coords_scale_;
  out.y = static_cast<double>(planning_graph_.map.parsed_map.height() - pos.y) /
          image_coords_scale_;
  return out;
}

} // namespace path_planner
