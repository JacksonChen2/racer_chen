#pragma once

#include "racer_3d_cpp/types.hpp"

#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace racer_3d_cpp {

inline constexpr double DRONE_RADIUS = 0.12;

struct Box3D {
  Point3 center{Point3::Zero()};
  Point3 size{Point3::Zero()};
  std::string name{"box"};

  [[nodiscard]] Point3 minimum() const { return center - 0.5 * size; }
  [[nodiscard]] Point3 maximum() const { return center + 0.5 * size; }
  [[nodiscard]] bool contains(
      const Point3 &point, double margin = 0.0) const;
};

struct Scenario3D {
  std::string name;
  Point3 map_min{Point3::Zero()};
  Point3 map_max{Point3::Zero()};
  std::vector<Point3> starts;
  std::vector<Box3D> obstacles;
  Point3 coarse_grid_size{5.0, 4.5, 2.0};
  std::string truth_mode{"analytic_boxes"};
  std::optional<Point3> safety_min;
  std::optional<Point3> safety_max;

  [[nodiscard]] Point3 mapSize() const { return map_max - map_min; }
};

[[nodiscard]] Scenario3D acceptanceScene();
[[nodiscard]] Scenario3D largeAcceptanceScene();
[[nodiscard]] Scenario3D warehouseSimpleScene();
[[nodiscard]] Scenario3D warehouseLoadedScene();
[[nodiscard]] Scenario3D getScenario(const std::string &name);

[[nodiscard]] double pointBoxSignedClearance(
    const Point3 &point, const Box3D &box);
[[nodiscard]] double obstacleClearance(
    const Point3 &point, const std::vector<Box3D> &obstacles);
[[nodiscard]] double rayBoxDistance(
    const Point3 &origin, const Point3 &direction, const Box3D &box);

[[nodiscard]] std::vector<Point3> simulatePointCloud(
    const Point3 &position, int azimuth_count = 90,
    int elevation_count = 15,
    double vertical_fov = 1.7453292519943295,
    double maximum_range = 7.0);
[[nodiscard]] std::vector<Point3> simulatePointCloud(
    const Point3 &position, int azimuth_count, int elevation_count,
    double vertical_fov, double maximum_range,
    const std::vector<Box3D> &obstacles);

[[nodiscard]] std::vector<double> pairwiseDistances(
    const std::vector<Point3> &points);

}  // namespace racer_3d_cpp
