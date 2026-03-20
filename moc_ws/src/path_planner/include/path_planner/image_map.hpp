#ifndef IMAGE_MAP_HPP
#define IMAGE_MAP_HPP

// Third-party
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <robot_utils/geometry.hpp>
#include <vector>

// Local libraries
#include "path_planner/cvParseMap2d.hpp"

namespace path_planner {

enum class MapBoundary { LEFT, TOP, RIGHT, BOTTOM };

template <typename CoordType> class ImageMap {
public:
  ImageMap();
  void loadMap(std::string map_image_filename, bool representative_points,
               double inflation_radius = 0.0);
  void initializePlot(int scale);
  bool
  inWorkspace(const robot_utils::geometry::Coords<CoordType> &coords) const;
  robot_utils::geometry::Coords<CoordType>
  push_inside_boundary(const robot_utils::geometry::Coords<CoordType> &coords,
                       double d) const;
  robot_utils::geometry::Coords<CoordType>
  push_along_boundary(const robot_utils::geometry::Coords<CoordType> &coords,
                      double d) const;
  bool
  isObstacleFree(const robot_utils::geometry::Coords<CoordType> &coords) const;
  std::unordered_map<robot_utils::geometry::Coords<CoordType>, double>
  allNeighbors(const robot_utils::geometry::Coords<CoordType> &coords) const;
  std::unordered_map<robot_utils::geometry::Coords<CoordType>, double>
  validNeighbors(const robot_utils::geometry::Coords<CoordType> &coords) const;
  cv::Point originalToDisplay(
      const robot_utils::geometry::Coords<CoordType> coords) const;
  void colorCoords(const robot_utils::geometry::Coords<CoordType> &coords,
                   cv::Scalar color);
  void plotPath(std::vector<robot_utils::geometry::Coords<CoordType>> &path);
  void renderDisplay(int pause = 1) const;
  cvParseMap2d parsed_map;

private:
  cv::Mat original_map;
  int displayScale;
  cv::Mat display_map;
  // robot_utils::geometry::Coords<CoordType> maxCoords;
  // robot_utils::geometry::Coords<CoordType> minCoords;
  static std::array<robot_utils::geometry::Coords<CoordType>, 4> directions4;
  static std::array<robot_utils::geometry::Coords<CoordType>, 8> directions8;

  std::vector<MapBoundary> get_boundary_violations(
      const robot_utils::geometry::Coords<CoordType> &coords) const;
};

template <typename CoordType>
std::array<robot_utils::geometry::Coords<CoordType>, 4>
    ImageMap<CoordType>::directions4 = {
        /* East, West, North, South */
        robot_utils::geometry::Coords<CoordType>{1, 0},
        robot_utils::geometry::Coords<CoordType>{-1, 0},
        robot_utils::geometry::Coords<CoordType>{0, -1},
        robot_utils::geometry::Coords<CoordType>{0, 1}};

template <typename CoordType>
std::array<robot_utils::geometry::Coords<CoordType>, 8>
    ImageMap<CoordType>::directions8 = {
        robot_utils::geometry::Coords<CoordType>{1, 0},   // East
        robot_utils::geometry::Coords<CoordType>{1, -1},  // South-East
        robot_utils::geometry::Coords<CoordType>{0, -1},  // South
        robot_utils::geometry::Coords<CoordType>{-1, -1}, // South-West
        robot_utils::geometry::Coords<CoordType>{-1, 0},  // West
        robot_utils::geometry::Coords<CoordType>{-1, 1},  // North-West
        robot_utils::geometry::Coords<CoordType>{0, 1},   // North
        robot_utils::geometry::Coords<CoordType>{1, 1},   // North-East
};

template <typename CoordType> ImageMap<CoordType>::ImageMap() {}

template <typename CoordType>
void ImageMap<CoordType>::loadMap(std::string map_image_filename,
                                  bool representative_points,
                                  double inflation_radius) {
  parsed_map = cvParseMap2d(map_image_filename, representative_points);
  parsed_map.minkowskiSum(inflation_radius);
  // maxCoords = robot_utils::geometry::Coords<CoordType>(parsed_map.width(),
  //                                                      parsed_map.height());
  // minCoords = robot_utils::geometry::Coords<CoordType>(0, 0);
}

template <typename CoordType>
void ImageMap<CoordType>::initializePlot(int scale) {
  displayScale = scale;
  // original_map = parsed_map.getCvMat(COLOR_MAP);
  original_map = parsed_map.getCvMat_inflated(COLOR_MAP);
  display_map = original_map.clone();
  cv::resize(display_map, display_map, cv::Size(), displayScale, displayScale);
  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  renderDisplay(3);
}

template <typename CoordType>
std::vector<MapBoundary> ImageMap<CoordType>::get_boundary_violations(
    const robot_utils::geometry::Coords<CoordType> &coords) const {
  std::vector<MapBoundary> res;
  if (coords.x < 0) {
    res.push_back(MapBoundary::LEFT);
  }
  if (coords.y < 0) {
    res.push_back(MapBoundary::TOP);
  }
  if (coords.x >= parsed_map.map.cols) {
    res.push_back(MapBoundary::RIGHT);
  }
  if (coords.y >= parsed_map.map.rows) {
    res.push_back(MapBoundary::BOTTOM);
  }
  return res;
}

template <typename CoordType>
robot_utils::geometry::Coords<CoordType>
ImageMap<CoordType>::push_inside_boundary(
    const robot_utils::geometry::Coords<CoordType> &coords, double d) const {
  robot_utils::geometry::Coords<CoordType> new_coords = coords;
  auto violations = get_boundary_violations(coords);
  for (const auto &violation : violations) {
    switch (violation) {
    case MapBoundary::LEFT:
      new_coords.x = d;
      break;
    case MapBoundary::TOP:
      new_coords.y = d;
      break;
    case MapBoundary::RIGHT:
      new_coords.x = parsed_map.map.cols - 1 - d;
      break;
    case MapBoundary::BOTTOM:
      new_coords.y = parsed_map.map.rows - 1 - d;
      break;
    }
  }
  return new_coords;
}

template <typename CoordType>
robot_utils::geometry::Coords<CoordType>
ImageMap<CoordType>::push_along_boundary(
    const robot_utils::geometry::Coords<CoordType> &coords, double d) const {
  robot_utils::geometry::Coords<CoordType> new_coords = coords;
  auto violations = get_boundary_violations(coords);
  for (const auto &violation : violations) {
    switch (violation) {
    case MapBoundary::LEFT:
      new_coords.y -= d;
      break;
    case MapBoundary::TOP:
      new_coords.x += d;
      break;
    case MapBoundary::RIGHT:
      new_coords.y += d;
      break;
    case MapBoundary::BOTTOM:
      new_coords.x -= d;
      break;
    }
  }
  return new_coords;
}

template <typename CoordType>
bool ImageMap<CoordType>::inWorkspace(
    const robot_utils::geometry::Coords<CoordType> &coords) const {
  // return minCoords.x <= coords.x && coords.x < maxCoords.x &&
  //        minCoords.y <= coords.y && coords.y < maxCoords.y;
  return parsed_map.isInFrame(coords.x, coords.y);
}

template <typename CoordType>
bool ImageMap<CoordType>::isObstacleFree(
    const robot_utils::geometry::Coords<CoordType> &coords) const {
  return parsed_map.isFree(coords.x, coords.y, true);
}

template <typename CoordType>
std::unordered_map<robot_utils::geometry::Coords<CoordType>, double>
ImageMap<CoordType>::allNeighbors(
    const robot_utils::geometry::Coords<CoordType> &coords) const {
  std::unordered_map<robot_utils::geometry::Coords<CoordType>, double>
      neighbors;

  for (auto dir : directions8) {
    robot_utils::geometry::Coords<CoordType> next{coords.x + dir.x,
                                                  coords.y + dir.y};
    neighbors.insert(std::make_pair(
        next, robot_utils::geometry::euclideanDistance2D(coords, next)));
  }
  return neighbors;
}

template <typename CoordType>
std::unordered_map<robot_utils::geometry::Coords<CoordType>, double>
ImageMap<CoordType>::validNeighbors(
    const robot_utils::geometry::Coords<CoordType> &coords) const {
  std::unordered_map<robot_utils::geometry::Coords<CoordType>, double>
      neighbors;

  for (auto dir : directions8) {
    robot_utils::geometry::Coords<CoordType> next{coords.x + dir.x,
                                                  coords.y + dir.y};
    if (inWorkspace(next) && isObstacleFree(next)) {
      neighbors.insert(std::make_pair(
          next, robot_utils::geometry::euclideanDistance2D(coords, next)));
    }
  }

  return neighbors;
}

template <typename CoordType>
cv::Point ImageMap<CoordType>::originalToDisplay(
    const robot_utils::geometry::Coords<CoordType> coords) const {
  return (cv::Point((displayScale * (coords.x)), (displayScale * (coords.y))));
}

template <typename CoordType>
void ImageMap<CoordType>::colorCoords(
    const robot_utils::geometry::Coords<CoordType> &coords, cv::Scalar color) {
  cv::Point plotPt = originalToDisplay(coords);
  for (int i = plotPt.x; i <= plotPt.x + (displayScale - 1); i++) {
    for (int j = plotPt.y; j <= plotPt.y + (displayScale - 1); j++) {
      if (i < 0 || i >= display_map.cols || j < 0 || j >= display_map.rows)
        continue;
      display_map.at<cv::Vec3b>(j, i) = cv::Vec3b(
          (uchar)color.val[0], (uchar)color.val[1], (uchar)color.val[2]);
    }
  }
}

template <typename CoordType>
void ImageMap<CoordType>::plotPath(
    std::vector<robot_utils::geometry::Coords<CoordType>> &path) {
  for (size_t i{0}; i < path.size() - 1; i++) {
    cv::line(display_map, originalToDisplay(path[i]),
             originalToDisplay(path[i + 1]), cv::Scalar(200, 100, 100),
             displayScale);
  }
}

template <typename CoordType>
void ImageMap<CoordType>::renderDisplay(int pause) const {
  cv::imshow("Display window", display_map);
  cv::waitKey(pause);
}

} // namespace path_planner

#endif