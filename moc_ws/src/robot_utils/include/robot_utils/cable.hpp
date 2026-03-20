#ifndef CABLE_HPP
#define CABLE_HPP

// Standard libraries
#include <string>
#include <vector>

// Local libraries
#include "robot_utils/geometry.hpp"

namespace robot_utils::cable {

enum class ActionType { Progress, Interlace };
enum class CableEnd { Rear, Front };

ActionType action_type_from_char(const char ch);

struct Cable {
  std::string name;
  std::string rear_end_robot;
  std::string front_end_robot;
};

std::string get_robot_at_end(const Cable &cable, CableEnd end);

struct CableAction {
  ActionType type;
  CableEnd end; // Progress: Front, Interlace: Rear or Front
  std::vector<robot_utils::geometry::Coords<double>>
      points; // waypoints for a progress action, exit point for an interlace
              // action
  size_t vertex_id; // required for interlace cooperation detection
  robot_utils::geometry::Coords<double>
      vertex_coords; // required for interlace (to indicate the center)
  bool is_last; // required for interlace to move robot all the way to the exit
};

} // namespace robot_utils::cable

#endif