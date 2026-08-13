#include "racer_3d_cpp/scenario.hpp"
#include "racer_3d_cpp/voxel_map.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace racer_3d_cpp {

TEST(VoxelMap, IntegratesThreeDimensionalRaysAndExtractsFrontiers) {
  VoxelMap map(0.25, Point3(0.0, 0.0, 0.0), Point3(4.0, 4.0, 3.0));
  std::vector<Point3> points;
  std::vector<bool> hits;
  const Point3 sensor(1.0, 1.0, 1.0);
  for (int z = -3; z <= 3; ++z) {
    for (int y = -4; y <= 4; ++y) {
      points.emplace_back(3.0, 1.0 + 0.15 * y, 1.0 + 0.15 * z);
      hits.push_back(true);
    }
  }
  map.updatePointCloud(sensor, points, 4.0, hits);
  EXPECT_GT(map.coverage(), 0.0);
  EXPECT_FALSE(map.frontierClusters(1U).empty());
  const auto wall = map.worldToGrid(Point3(3.0, 1.0, 1.0));
  ASSERT_TRUE(wall.has_value());
  EXPECT_EQ(map.stateAt(*wall), OCCUPIED);
}

TEST(VoxelMap, EsdfIsMetricAndTrilinear) {
  VoxelMap map(0.2, Point3::Zero(), Point3(3.0, 3.0, 3.0));
  std::vector<std::int8_t> states(map.voxelCount(), FREE);
  const GridIndex3 occupied{7, 7, 7};
  states[map.flatIndex(occupied)] = OCCUPIED;
  map.setStates(states);
  EXPECT_LT(map.distanceAt(map.gridToWorld(occupied), false), 0.0);
  const double distance =
      map.distanceAt(map.gridToWorld({10, 7, 7}), false);
  EXPECT_NEAR(distance, 0.6, 0.11);
  EXPECT_GT(map.esdfGradient(map.gridToWorld({10, 7, 7}), false).x(), 0.0);
}

TEST(Scenario, WarehouseDefinesFiveCollisionSeparatedStarts) {
  const auto scenario = warehouseSimpleScene();
  ASSERT_EQ(scenario.starts.size(), 5U);
  EXPECT_EQ(scenario.truth_mode, "observed_volume");
  EXPECT_TRUE(scenario.safety_min.has_value());
  const auto distances = pairwiseDistances(scenario.starts);
  ASSERT_FALSE(distances.empty());
  EXPECT_GT(*std::min_element(distances.begin(), distances.end()), 0.35);
}

TEST(Scenario, WarehouseLoadedCoversGeneratedCargoZone) {
  const auto scenario = warehouseLoadedScene();
  EXPECT_EQ(scenario.name, "warehouse_loaded");
  EXPECT_TRUE(scenario.truth_mode == "observed_volume");
  ASSERT_EQ(scenario.starts.size(), 3U);
  EXPECT_NEAR((scenario.map_max - scenario.map_min).x(), 32.6, 1.0e-9);
  EXPECT_NEAR((scenario.map_max - scenario.map_min).y(), 19.0, 1.0e-9);
  EXPECT_NEAR((scenario.map_max - scenario.map_min).z(), 8.5, 1.0e-9);
  const Point3 cargo_min(-25.3773, 9.4816, 1.5957);
  const Point3 cargo_max(4.4511, 24.2265, 6.4522);
  EXPECT_TRUE((cargo_min.array() > scenario.map_min.array()).all());
  EXPECT_TRUE((cargo_max.array() < scenario.map_max.array()).all());
  for (const auto &start : scenario.starts) {
    EXPECT_TRUE((start.array() > scenario.map_min.array()).all());
    EXPECT_TRUE((start.array() < scenario.map_max.array()).all());
  }
}

}  // namespace racer_3d_cpp
