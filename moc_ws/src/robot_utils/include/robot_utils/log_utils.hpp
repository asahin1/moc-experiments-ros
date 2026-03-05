#ifndef ROBOT_UTILS_LOG_UTILS_HPP
#define ROBOT_UTILS_LOG_UTILS_HPP

#include <string>

#include "robot_utils/cable.hpp"

namespace robot_utils::log {

inline std::string get_robot_prefix(const std::string &robot_name) {
  return std::string("[") + robot_name + "]";
}

inline std::string get_cable_prefix(const cable::Cable &cable) {
  return std::string("[") + cable.name + "]";
}
} // namespace robot_utils::log

#endif