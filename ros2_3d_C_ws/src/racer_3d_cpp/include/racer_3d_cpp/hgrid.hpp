#pragma once

#include "racer_3d_cpp/types.hpp"

#include <array>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace racer_3d_cpp {

class VoxelMap;

struct HGridCell3D {
  int level{};
  int ix{};
  int iy{};
  int iz{};
  // Half-open voxel bounds: [x0, x1) x [y0, y1) x [z0, z1).
  std::array<int, 6> bounds{};
  Point3 centroid{Point3::Zero()};
  int unknownCount{};
  int total{};

  std::string id() const;
  int demand() const { return unknownCount; }
};

using HGridDistanceFunction =
    std::function<double(const Point3 &, const Point3 &)>;

class HierarchicalGrid3D {
 public:
  using CellMap = std::map<std::string, HGridCell3D>;

  explicit HierarchicalGrid3D(
      const VoxelMap &voxel_map,
      const Point3 &coarse_size = Point3(5.0, 4.5, 2.0), int levels = 2,
      double subdivide_known_ratio = 0.35, int minimum_unknown = 4);

  const CellMap &update();
  const CellMap &cells() const { return cells_; }

  std::optional<HGridCell3D> containing(const Point3 &point) const;
  std::optional<HGridCell3D> containing(
      const Point3 &point, const std::vector<HGridCell3D> &candidates) const;

  std::map<std::string, int> initialOwners(
      const std::vector<Point3> &starts) const;

  std::vector<std::string> coverageRoute(
      const std::vector<std::string> &cell_ids, const Point3 &start,
      const HGridDistanceFunction &distance_function = {}) const;

 private:
  std::array<int, 3> span(int level) const;
  HGridCell3D cellFromIndex(int level, int ix, int iy, int iz) const;
  std::vector<HGridCell3D> twoOpt(
      const std::vector<HGridCell3D> &route, const Point3 &start,
      const HGridDistanceFunction &distance_function) const;

  const VoxelMap &map_;
  Point3 coarse_size_;
  int levels_;
  double subdivide_known_ratio_;
  int minimum_unknown_;
  CellMap cells_;
  std::vector<std::string> cell_order_;
};

}  // namespace racer_3d_cpp
