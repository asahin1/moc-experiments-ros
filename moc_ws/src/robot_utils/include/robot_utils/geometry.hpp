#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

// Standard libraries
#include <iostream>
#include <math.h>
#include <type_traits>

namespace robot_utils::geometry {

template <typename CoordType> struct Coords {
  static_assert(std::is_arithmetic_v<CoordType>,
                "Coords requires an arithmetic type");
  CoordType x;
  CoordType y;
  Coords() : x{0}, y{0} {}
  Coords(CoordType xx, CoordType yy) : x(xx), y(yy) {}

  double length() const { return sqrt(x * x + y * y); }

  Coords normalized() const {
    double len = length();
    return (len == 0) ? Coords(0, 0) : Coords(x / len, y / len);
  }

  inline double angleOf() const {
    double a = std::atan2(y, x);
    return (a < 0) ? a + 2 * M_PI : a;
  }

  void rotate(double radians) {
    double c = std::cos(radians);
    double s = std::sin(radians);
    double nx = c * x - s * y;
    double ny = s * x + c * y;
    x = nx;
    y = ny;
  }
};

template <typename CoordType1, typename CoordType2>
inline Coords<double> operator+(const Coords<CoordType1> &a,
                                const Coords<CoordType2> &b) {
  Coords<double> res{a.x + b.x, a.y + b.y};
  return res;
}

template <typename CoordType1, typename CoordType2>
inline Coords<double> operator-(const Coords<CoordType1> &a,
                                const Coords<CoordType2> &b) {
  Coords<double> res{a.x - b.x, a.y - b.y};
  return res;
}

template <typename CoordType, typename ScaleType>
inline Coords<CoordType> operator*(const Coords<CoordType> &a,
                                   const ScaleType &b) {
  Coords<double> res{a.x * b, a.y * b};
  return res;
}

template <typename CoordType, typename ScaleType>
inline Coords<CoordType> operator*(const ScaleType &b,
                                   const Coords<CoordType> &a) {
  Coords<double> res{a.x * b, a.y * b};
  return res;
}

// Dot product
template <typename CoordType>
inline double dot(const Coords<CoordType> &a, const Coords<CoordType> &b) {
  return a.x * b.x + a.y * b.y;
}

// Cross product magnitude (2D)
template <typename CoordType>
inline double cross(const Coords<CoordType> &a, const Coords<CoordType> &b) {
  return a.x * b.y - a.y * b.x;
}

template <typename CoordType>
double signed_angle(const Coords<CoordType> &a, const Coords<CoordType> &b) {
  double crossVal = cross(a, b); // positive => CCW, negative => CW
  double dotVal = dot(a, b);
  return std::atan2(crossVal, dotVal); // range: (-π, +π]
}

template <typename CoordType>
inline bool operator==(const Coords<CoordType> a, const Coords<CoordType> b) {
  return a.x == b.x && a.y == b.y;
}

template <typename CoordType>
inline bool operator!=(const Coords<CoordType> a, const Coords<CoordType> b) {
  return !(a == b);
}

template <typename CoordType>
inline std::basic_iostream<char>::basic_ostream &
operator<<(std::ostream &os, const Coords<CoordType> &c) {
  os << "x: " << c.x << ", y: " << c.y << std::endl;
  return os;
}

template <typename CoordType1, typename CoordType2>
inline double euclideanDistance2D(CoordType1 p1, CoordType2 p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return sqrt(dx * dx + dy * dy);
}

template <typename CoordType1, typename CoordType2>
inline double manhattanDistance2D(CoordType1 p1, CoordType2 p2) {
  double dx = abs(p1.x - p2.x);
  double dy = abs(p1.y - p2.y);
  return dx + dy;
}

struct Pose2D {
  Coords<double> position;
  Coords<double> heading;
};

} // namespace robot_utils::geometry

template <typename CoordType>
struct std::hash<robot_utils::geometry::Coords<CoordType>> {
  std::size_t
  operator()(const robot_utils::geometry::Coords<CoordType> &s) const noexcept {
    return (s.x ^ (s.y << 16));
  }
};

#endif