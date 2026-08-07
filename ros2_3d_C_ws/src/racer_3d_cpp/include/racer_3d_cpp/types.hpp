#pragma once

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace racer_3d_cpp {

constexpr std::int8_t UNKNOWN = -1;
constexpr std::int8_t FREE = 0;
constexpr std::int8_t OCCUPIED = 100;

struct GridIndex3 {
  int x{};
  int y{};
  int z{};

  bool operator==(const GridIndex3 &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const GridIndex3 &other) const { return !(*this == other); }
  bool operator<(const GridIndex3 &other) const {
    if (z != other.z) return z < other.z;
    if (y != other.y) return y < other.y;
    return x < other.x;
  }
};

struct GridIndex3Hash {
  std::size_t operator()(const GridIndex3 &cell) const noexcept {
    std::size_t value = static_cast<std::size_t>(cell.x) * 73856093U;
    value ^= static_cast<std::size_t>(cell.y) * 19349663U;
    value ^= static_cast<std::size_t>(cell.z) * 83492791U;
    return value;
  }
};

using Point3 = Eigen::Vector3d;

struct TrajectorySample {
  double time{};
  Point3 position{Point3::Zero()};
};

struct PeerState {
  int drone_id{-1};
  double stamp{};
  double received{};
  Point3 position{Point3::Zero()};
  Point3 velocity{Point3::Zero()};
  std::vector<std::string> owned_cells;
  std::vector<TrajectorySample> trajectory;
};

}  // namespace racer_3d_cpp
