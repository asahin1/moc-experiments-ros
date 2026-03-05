#ifndef CABLE_HPP
#define CABLE_HPP

// Standard libraries
#include <string>
#include <vector>

// Local libraries
#include "robot_utils/geometry.hpp"

namespace robot_utils::cable {

enum class ActionType { Move, Hitch };
enum class CableEnd { First, Last };

ActionType action_type_from_char(const char ch);

struct Cable {
  std::string name;
  std::string first_end_robot;
  std::string last_end_robot;
};

std::string get_robot_at_end(const Cable &cable, CableEnd end);

struct CableAction {
  ActionType type;
  CableEnd end; // Move: Last, Hitch: First or Last
  std::vector<robot_utils::geometry::Coords<double>>
      points; // waypoints for a move action, exit point for a hitch action
  size_t vertex_id; // required for hitch cooperation detection
  robot_utils::geometry::Coords<double>
      vertex_coords; // required for hitches (to indicate the center)
  bool is_last; // required for hitches to move robot all the way to the exit
};

} // namespace robot_utils::cable

#endif