#include "racer_3d_cpp/hgrid.hpp"
#include "racer_3d_cpp/planning.hpp"
#include "racer_3d_cpp/voxel_map.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace racer_3d_cpp {

TEST(Planning, AStarUsesVerticalPassageWithoutCornerCutting) {
  constexpr int nx = 9;
  constexpr int ny = 7;
  constexpr int nz = 5;
  std::vector<std::uint8_t> blocked(nx * ny * nz, 0U);
  auto index = [](int x, int y, int z) {
    return static_cast<std::size_t>(x + nx * (y + ny * z));
  };
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < ny; ++y) blocked[index(4, y, z)] = 1U;
  }
  const auto path = astar3d(blocked, nx, ny, nz, {1, 3, 1}, {7, 3, 1}, 0.2);
  ASSERT_FALSE(path.empty());
  int highest = 0;
  for (const auto &cell : path) highest = std::max(highest, cell.z);
  EXPECT_GE(highest, 3);
}

TEST(Planning, BSplineKeepsXYZEndpointsAndFiniteTiming) {
  VoxelMap map(0.2, Point3::Zero(), Point3(5.0, 5.0, 3.0));
  map.setStates(std::vector<std::int8_t>(map.voxelCount(), FREE));
  const std::vector<Point3> points{
      Point3(0.5, 0.5, 0.5), Point3(1.5, 1.0, 1.0),
      Point3(2.5, 2.0, 1.8), Point3(4.0, 3.5, 2.2)};
  const auto result =
      minimumTimeBsplineTrajectory(points, map, 0.15, 0.8, 1.5, 0.1);
  ASSERT_GT(result.samples.size(), 2U);
  EXPECT_NEAR((result.samples.front().position - points.front()).norm(), 0.0,
              1.0e-8);
  EXPECT_NEAR((result.samples.back().position - points.back()).norm(), 0.0,
              1.0e-8);
  EXPECT_GT(result.samples.back().time, 0.0);
}

}  // namespace racer_3d_cpp
