#pragma once

#include "racer_3d_cpp/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace racer_3d_cpp {

/**
 * Probabilistic three-dimensional occupancy map.
 *
 * Storage is contiguous in (z, y, x) order, matching the Python RACER 3-D
 * implementation and its ROS map-sharing wire format.
 */
class VoxelMap {
 public:
  VoxelMap(double resolution, const Point3 &origin, const Point3 &size);

  [[nodiscard]] double resolution() const noexcept { return resolution_; }
  [[nodiscard]] const Point3 &origin() const noexcept { return origin_; }
  [[nodiscard]] const Point3 &size() const noexcept { return size_; }
  [[nodiscard]] int nx() const noexcept { return nx_; }
  [[nodiscard]] int ny() const noexcept { return ny_; }
  [[nodiscard]] int nz() const noexcept { return nz_; }
  [[nodiscard]] std::size_t voxelCount() const noexcept {
    return log_odds_.size();
  }

  [[nodiscard]] bool inBounds(const GridIndex3 &cell) const noexcept;
  [[nodiscard]] std::size_t flatIndex(const GridIndex3 &cell) const noexcept;
  [[nodiscard]] std::optional<GridIndex3> worldToGrid(
      const Point3 &point) const;
  [[nodiscard]] Point3 gridToWorld(const GridIndex3 &cell) const;

  [[nodiscard]] std::vector<GridIndex3> rayCells(
      const Point3 &start, const Point3 &end) const;

  void updatePointCloud(
      const Point3 &sensor_origin, const std::vector<Point3> &points_world,
      double maximum_range,
      const std::vector<std::uint8_t> &hit_mask = {},
      std::size_t maximum_rays = 2400U);
  void updatePointCloud(
      const Point3 &sensor_origin, const std::vector<Point3> &points_world,
      double maximum_range, const std::vector<bool> &hit_mask,
      std::size_t maximum_rays = 2400U);

  [[nodiscard]] const std::vector<std::int8_t> &states() const;
  [[nodiscard]] std::int8_t stateAt(const GridIndex3 &cell) const;
  void setStates(const std::vector<std::int8_t> &states);
  void merge(const std::vector<std::int8_t> &states);
  [[nodiscard]] double coverage() const;

  /**
   * Signed Euclidean distance field in metres.
   *
   * Positive values are outside the selected source set and negative values
   * are inside it. The field is computed with an O(N) separable exact squared
   * Euclidean distance transform and cached until the map changes.
   */
  [[nodiscard]] const std::vector<float> &esdf(
      bool unknown_is_occupied = true) const;
  [[nodiscard]] double distanceAt(
      const Point3 &point, bool unknown_is_occupied = true) const;
  [[nodiscard]] Point3 esdfGradient(
      const Point3 &point, bool unknown_is_occupied = true) const;
  [[nodiscard]] std::vector<std::uint8_t> inflatedBlocked(
      double clearance, bool unknown_is_blocked = true) const;

  [[nodiscard]] std::vector<std::uint8_t> frontierMask() const;
  [[nodiscard]] std::vector<std::vector<GridIndex3>> frontierClusters(
      std::size_t minimum_size = 4U) const;
  [[nodiscard]] int informationGain(
      const Point3 &point, double radius_m = 2.5) const;
  [[nodiscard]] int visibleUnknownGain(
      const Point3 &viewpoint, const std::vector<GridIndex3> &cluster,
      std::size_t maximum_rays = 48U) const;

  [[nodiscard]] const std::vector<std::int16_t> &logOdds() const noexcept {
    return log_odds_;
  }
  [[nodiscard]] const std::vector<std::uint16_t> &observations() const noexcept {
    return observations_;
  }

 private:
  void observeBulk(std::vector<std::size_t> cells, int delta);
  void invalidateCaches() noexcept;
  [[nodiscard]] std::vector<float> squaredDistanceTransform(
      const std::vector<std::uint8_t> &source) const;

  double resolution_;
  Point3 origin_;
  Point3 size_;
  int nx_;
  int ny_;
  int nz_;
  std::vector<std::int16_t> log_odds_;
  std::vector<std::uint16_t> observations_;

  mutable bool states_valid_{false};
  mutable std::vector<std::int8_t> states_cache_;
  mutable bool esdf_unknown_valid_{false};
  mutable bool esdf_known_valid_{false};
  mutable std::vector<float> esdf_unknown_cache_;
  mutable std::vector<float> esdf_known_cache_;
};

}  // namespace racer_3d_cpp
