#pragma once

#include "racer_3d_cpp/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace racer_3d_cpp {

class HierarchicalGrid3D;
class VoxelMap;

struct ExplorationPlan3D {
  Point3 goal{Point3::Zero()};
  double yaw{};
  double pitch{};
  Point3 frontier_centroid{Point3::Zero()};
  std::vector<GridIndex3> grid_path;
  std::vector<Point3> geometric_path;
  std::vector<TrajectorySample> trajectory;
  int information_gain{};
  double minimum_clearance{};
};

struct TrajectoryResult {
  std::vector<TrajectorySample> samples;
  double minimum_clearance{};
};

// ``blocked`` is flattened as x + nx * (y + ny * z).
std::vector<GridIndex3> astar3d(const std::vector<std::uint8_t> &blocked,
                                int nx, int ny, int nz, const GridIndex3 &start,
                                const GridIndex3 &goal,
                                double resolution = 1.0);

std::vector<GridIndex3> shortenPath3d(const VoxelMap &voxel_map,
                                      const std::vector<GridIndex3> &path,
                                      const std::vector<std::uint8_t> &blocked);

class UniformBSpline3D {
public:
  explicit UniformBSpline3D(const std::vector<Point3> &control_points,
                            int degree = 3);

  Point3 evaluate(double parameter) const;
  const std::vector<Point3> &controlPoints() const { return points_; }
  int degree() const { return degree_; }

private:
  std::vector<Point3> points_;
  std::vector<double> knots_;
  int degree_{3};
};

std::vector<Point3>
optimizeBsplineControlPoints(const std::vector<Point3> &points,
                             const VoxelMap &voxel_map, double clearance,
                             int iterations = 35);

std::vector<Point3>
optimizeBSplineControlPoints(const std::vector<Point3> &points,
                             const VoxelMap &voxel_map, double clearance,
                             int iterations = 35);

TrajectoryResult minimumTimeBsplineTrajectory(const std::vector<Point3> &points,
                                              const VoxelMap &voxel_map,
                                              double clearance,
                                              double max_speed,
                                              double max_acceleration,
                                              double sample_dt = 0.15);

std::pair<std::vector<TrajectorySample>, double>
minimumTimeBSplineTrajectory(const std::vector<Point3> &points,
                             const VoxelMap &voxel_map, double clearance,
                             double max_speed, double max_acceleration,
                             double sample_dt = 0.15);

std::optional<ExplorationPlan3D>
planExploration(const VoxelMap &voxel_map, const HierarchicalGrid3D &hgrid,
                const std::vector<std::string> &owned_cells,
                const std::vector<std::string> &coverage_route,
                const Point3 &position, double clearance, double max_speed,
                double max_acceleration);

} // namespace racer_3d_cpp
