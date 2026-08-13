#include "racer_3d_cpp/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace racer_3d_cpp {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

std::vector<Box3D> roomBoundaries(
    double half_x, double half_y, double height, double thickness = 0.12) {
  return {
      {{0.0, 0.0, -0.5 * thickness},
       {2.0 * half_x, 2.0 * half_y, thickness}, "floor"},
      {{0.0, 0.0, height + 0.5 * thickness},
       {2.0 * half_x, 2.0 * half_y, thickness}, "ceiling"},
      {{-half_x - 0.5 * thickness, 0.0, 0.5 * height},
       {thickness, 2.0 * half_y, height}, "west_wall"},
      {{half_x + 0.5 * thickness, 0.0, 0.5 * height},
       {thickness, 2.0 * half_y, height}, "east_wall"},
      {{0.0, -half_y - 0.5 * thickness, 0.5 * height},
       {2.0 * half_x, thickness, height}, "south_wall"},
      {{0.0, half_y + 0.5 * thickness, 0.5 * height},
       {2.0 * half_x, thickness, height}, "north_wall"},
  };
}

void appendBoxes(
    std::vector<Box3D> &destination, std::vector<Box3D> additions) {
  destination.insert(
      destination.end(),
      std::make_move_iterator(additions.begin()),
      std::make_move_iterator(additions.end()));
}

}  // namespace

bool Box3D::contains(const Point3 &point, double margin) const {
  const Point3 lower = minimum();
  const Point3 upper = maximum();
  for (int axis = 0; axis < 3; ++axis) {
    if (point[axis] < lower[axis] - margin ||
        point[axis] > upper[axis] + margin) {
      return false;
    }
  }
  return true;
}

Scenario3D acceptanceScene() {
  constexpr double height = 2.0;
  auto obstacles = roomBoundaries(7.5, 4.5, height);
  appendBoxes(
      obstacles,
      {
          {{-1.8, -2.0, 0.52}, {0.40, 4.6, 1.04}, "low_partition"},
          {{1.8, 2.0, 1.55}, {0.40, 4.6, 0.90}, "high_partition"},
          {{-4.2, 2.6, 0.75}, {1.00, 1.00, 1.50}, "west_column"},
          {{0.0, 0.0, 1.00}, {1.10, 1.10, 2.00}, "center_column"},
          {{4.4, -2.6, 0.60}, {1.20, 1.00, 1.20}, "east_low_block"},
          {{5.2, 2.4, 1.30}, {0.90, 1.10, 1.40},
           "east_hanging_block"},
      });
  Scenario3D scenario;
  scenario.name = "acceptance_15x9x2";
  scenario.map_min = Point3(-7.5, -4.5, 0.0);
  scenario.map_max = Point3(7.5, 4.5, 2.0);
  scenario.starts = {
      {-6.4, -3.2, 0.45}, {-6.4, 0.0, 1.00}, {-6.4, 3.2, 1.55}};
  scenario.obstacles = std::move(obstacles);
  scenario.coarse_grid_size = Point3(5.0, 4.5, 2.0);
  return scenario;
}

Scenario3D largeAcceptanceScene() {
  constexpr double height = 3.0;
  auto obstacles = roomBoundaries(10.0, 25.0, height);
  appendBoxes(
      obstacles,
      {
          {{0.0, -15.0, 0.65}, {20.0, 0.45, 1.30},
           "south_low_overflight_wall"},
          {{0.0, -5.0, 2.20}, {20.0, 0.45, 1.60},
           "south_high_underflight_wall"},
          {{-5.5, 5.0, 1.50}, {9.0, 0.45, 3.0}, "center_gate_left"},
          {{5.5, 5.0, 1.50}, {9.0, 0.45, 3.0}, "center_gate_right"},
          {{-5.0, 15.0, 0.70}, {10.0, 0.45, 1.40}, "north_low_half"},
          {{5.0, 15.0, 2.25}, {10.0, 0.45, 1.50}, "north_high_half"},
          {{-6.2, -20.0, 1.50}, {1.20, 1.20, 3.0},
           "column_south_west"},
          {{6.0, -10.0, 1.50}, {1.10, 1.10, 3.0},
           "column_south_east"},
          {{-5.8, 0.0, 1.50}, {1.20, 1.20, 3.0},
           "column_center_west"},
          {{5.8, 10.0, 1.50}, {1.20, 1.20, 3.0},
           "column_north_east"},
          {{-6.0, 20.0, 1.50}, {1.10, 1.10, 3.0},
           "column_north_west"},
          {{3.0, -20.0, 0.60}, {1.8, 1.6, 1.20}, "south_low_block"},
          {{-2.5, -10.0, 2.20}, {1.8, 1.6, 1.60},
           "south_hanging_block"},
          {{3.0, 0.0, 0.75}, {1.8, 1.8, 1.50}, "center_low_block"},
          {{-3.0, 10.0, 2.15}, {1.8, 1.8, 1.70},
           "north_hanging_block"},
          {{3.5, 21.0, 0.70}, {2.0, 1.6, 1.40}, "north_low_block"},
      });
  Scenario3D scenario;
  scenario.name = "acceptance_20x50x3";
  scenario.map_min = Point3(-10.0, -25.0, 0.0);
  scenario.map_max = Point3(10.0, 25.0, 3.0);
  scenario.starts = {
      {-6.0, -23.0, 0.60}, {0.0, -23.0, 1.50}, {6.0, -23.0, 2.40}};
  scenario.obstacles = std::move(obstacles);
  scenario.coarse_grid_size = Point3(5.0, 10.0, 3.0);
  return scenario;
}

Scenario3D warehouseSimpleScene() {
  Scenario3D scenario;
  scenario.name = "warehouse_simple";
  scenario.map_min = Point3(-10.2, -12.0, 0.0);
  scenario.map_max = Point3(9.2, 17.8, 9.0);
  scenario.starts = {
      {-6.0, -10.0, 0.60},
      {0.0, -10.0, 1.50},
      {6.0, -10.0, 2.40},
      {-3.0, -10.0, 1.05},
      {3.0, -10.0, 1.95},
  };
  scenario.coarse_grid_size = Point3(4.85, 7.45, 4.50);
  scenario.truth_mode = "observed_volume";
  scenario.safety_min = Point3(-10.37, -12.24, 0.0);
  scenario.safety_max = Point3(9.37, 17.96, 9.0);
  return scenario;
}

Scenario3D warehouseLoadedScene() {
  Scenario3D scenario;
  scenario.name = "warehouse_loaded";
  scenario.map_min = Point3(-26.8, 7.2, 0.0);
  scenario.map_max = Point3(5.8, 26.2, 8.5);
  scenario.starts = {
      {-24.0, 8.0, 0.80},
      {-10.5, 8.0, 1.50},
      {3.0, 8.0, 2.20},
  };
  scenario.coarse_grid_size = Point3(8.15, 4.75, 4.25);
  scenario.truth_mode = "observed_volume";
  scenario.safety_min = Point3(-27.05, 6.95, 0.0);
  scenario.safety_max = Point3(6.05, 26.45, 8.8);
  return scenario;
}

Scenario3D getScenario(const std::string &name) {
  if (name == "acceptance_15x9x2") {
    return acceptanceScene();
  }
  if (name == "acceptance_20x50x3") {
    return largeAcceptanceScene();
  }
  if (name == "warehouse_simple") {
    return warehouseSimpleScene();
  }
  if (name == "warehouse_loaded") {
    return warehouseLoadedScene();
  }
  throw std::invalid_argument(
      "unknown scenario '" + name +
      "'; choose acceptance_15x9x2, acceptance_20x50x3, warehouse_simple, "
      "or warehouse_loaded");
}

double pointBoxSignedClearance(const Point3 &point, const Box3D &box) {
  const Point3 lower = box.minimum();
  const Point3 upper = box.maximum();
  Point3 delta = Point3::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    delta[axis] = std::max(
        {lower[axis] - point[axis], 0.0, point[axis] - upper[axis]});
  }
  if ((delta.array() > 0.0).any()) {
    return delta.norm();
  }
  double penetration = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    penetration = std::min(
        penetration,
        std::min(point[axis] - lower[axis], upper[axis] - point[axis]));
  }
  return -penetration;
}

double obstacleClearance(
    const Point3 &point, const std::vector<Box3D> &obstacles) {
  double clearance = std::numeric_limits<double>::infinity();
  for (const auto &obstacle : obstacles) {
    clearance = std::min(
        clearance, pointBoxSignedClearance(point, obstacle));
  }
  return clearance;
}

double rayBoxDistance(
    const Point3 &origin, const Point3 &direction, const Box3D &box) {
  double t_min = -std::numeric_limits<double>::infinity();
  double t_max = std::numeric_limits<double>::infinity();
  const Point3 lower = box.minimum();
  const Point3 upper = box.maximum();
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < 1.0e-12) {
      if (origin[axis] < lower[axis] || origin[axis] > upper[axis]) {
        return std::numeric_limits<double>::infinity();
      }
      continue;
    }
    const double first = (lower[axis] - origin[axis]) / direction[axis];
    const double second = (upper[axis] - origin[axis]) / direction[axis];
    t_min = std::max(t_min, std::min(first, second));
    t_max = std::min(t_max, std::max(first, second));
    if (t_min > t_max) {
      return std::numeric_limits<double>::infinity();
    }
  }
  if (t_max < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, t_min);
}

std::vector<Point3> simulatePointCloud(
    const Point3 &position, int azimuth_count, int elevation_count,
    double vertical_fov, double maximum_range) {
  static const std::vector<Box3D> obstacles = acceptanceScene().obstacles;
  return simulatePointCloud(
      position, azimuth_count, elevation_count, vertical_fov, maximum_range,
      obstacles);
}

std::vector<Point3> simulatePointCloud(
    const Point3 &position, int azimuth_count, int elevation_count,
    double vertical_fov, double maximum_range,
    const std::vector<Box3D> &obstacles) {
  if (azimuth_count <= 0 || elevation_count <= 0) {
    return {};
  }
  std::vector<Point3> points;
  points.reserve(
      static_cast<std::size_t>(azimuth_count) *
      static_cast<std::size_t>(elevation_count));
  for (int elevation_index = 0; elevation_index < elevation_count;
       ++elevation_index) {
    const double ratio =
        elevation_count == 1
            ? 0.0
            : static_cast<double>(elevation_index) /
                  static_cast<double>(elevation_count - 1);
    const double elevation =
        -0.5 * vertical_fov + ratio * vertical_fov;
    const double cos_elevation = std::cos(elevation);
    for (int azimuth_index = 0; azimuth_index < azimuth_count;
         ++azimuth_index) {
      const double azimuth =
          -kPi + 2.0 * kPi * static_cast<double>(azimuth_index) /
                     static_cast<double>(azimuth_count);
      const Point3 direction(
          cos_elevation * std::cos(azimuth),
          cos_elevation * std::sin(azimuth),
          std::sin(elevation));
      double distance = maximum_range;
      for (const auto &obstacle : obstacles) {
        distance = std::min(
            distance, rayBoxDistance(position, direction, obstacle));
      }
      points.push_back(distance * direction);
    }
  }
  return points;
}

std::vector<double> pairwiseDistances(const std::vector<Point3> &points) {
  std::vector<double> distances;
  distances.reserve(points.size() * (points.size() - 1U) / 2U);
  for (std::size_t first = 0; first < points.size(); ++first) {
    for (std::size_t second = first + 1U; second < points.size(); ++second) {
      distances.push_back((points[first] - points[second]).norm());
    }
  }
  return distances;
}

}  // namespace racer_3d_cpp
