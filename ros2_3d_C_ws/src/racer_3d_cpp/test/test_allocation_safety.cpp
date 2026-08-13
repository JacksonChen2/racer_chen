#include "racer_3d_cpp/allocation.hpp"
#include "racer_3d_cpp/safety.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace racer_3d_cpp {

TEST(Allocation, PairwisePartitionIsCompleteAndBalanced) {
  std::vector<HGridCell3D> cells;
  for (int index = 0; index < 6; ++index) {
    HGridCell3D cell;
    cell.level = 1;
    cell.ix = index;
    cell.centroid = Point3(index * 1.0, 0.0, 1.0);
    cell.unknownCount = 20 + index;
    cell.total = 100;
    cells.push_back(cell);
  }
  const auto result =
      capacityPartition(cells, Point3(0.0, 0.0, 1.0),
                        Point3(6.0, 0.0, 1.0));
  EXPECT_EQ(result.first.size() + result.second.size(), cells.size());
  EXPECT_EQ(result.firstRoute.size(), result.first.size());
  EXPECT_EQ(result.secondRoute.size(), result.second.size());
  EXPECT_GT(result.firstDemand, 0);
  EXPECT_GT(result.secondDemand, 0);
}

TEST(Safety, CbfRemovesClosingVelocity) {
  PeerState peer;
  peer.drone_id = 1;
  peer.position = Point3(0.4, 0.0, 1.0);
  peer.velocity = Point3::Zero();
  const Point3 preferred(0.3, 0.0, 0.0);
  const Point3 safe = cbfSwarmFilter(
      preferred, Point3(0.0, 0.0, 1.0), {peer}, 0.65, 0.5);
  EXPECT_LT(safe.x(), preferred.x());
}

TEST(Safety, CbfReservesRigidBodyStoppingDistance) {
  PeerState peer;
  peer.drone_id = 1;
  peer.position = Point3(2.0, 0.0, 1.0);
  peer.velocity = Point3::Zero();
  const Point3 safe = cbfSwarmFilter(
      Point3(0.8, 0.0, 0.0), Point3(0.0, 0.0, 1.0), {peer}, 1.0, 0.85,
      Point3(1.0, 0.0, 0.0));
  EXPECT_LE(safe.x(), 0.0);
}

TEST(Safety, PointCloudBarrierBrakesBeforeBodyClearance) {
  const Point3 safe = pointcloudObstacleFilter(
      Point3(0.8, 0.0, 0.0), Point3::Zero(),
      {Point3(1.5, 0.0, 0.0)}, 0.5, 0.85,
      Point3(0.8, 0.0, 0.0));
  EXPECT_LT(safe.x(), 0.30);
}

TEST(Safety, DetectsSpaceTimeTrajectoryConflict) {
  const std::vector<TrajectorySample> first{
      {1.0, Point3(0.0, 0.0, 1.0)},
      {2.0, Point3(1.0, 0.0, 1.0)}};
  const std::vector<TrajectorySample> second{
      {1.1, Point3(0.2, 0.0, 1.0)},
      {3.0, Point3(4.0, 0.0, 1.0)}};
  EXPECT_TRUE(predictedPathConflict(first, second, 0.5));
}

}  // namespace racer_3d_cpp
