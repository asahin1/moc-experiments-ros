#include "robot_utils/cable.hpp"
#include <stdexcept>

namespace robot_utils::cable {
ActionType action_type_from_char(const char ch) {
  switch (ch) {
  case 'm':
    return ActionType::Progress;
  case 'h':
    return ActionType::Interlace;
  default:
    throw std::runtime_error("Unsupported action type: " + std::string(1, ch));
  }
}

std::string get_robot_at_end(const Cable &cable, CableEnd end) {
  switch (end) {
  case CableEnd::Rear:
    return cable.rear_end_robot;
  case CableEnd::Front:
    return cable.front_end_robot;
  default:
    throw std::runtime_error("Cable end unspecified");
  }
}
} // namespace robot_utils::cable