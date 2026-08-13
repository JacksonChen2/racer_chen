#pragma once

#include "racer_3d_cpp/types.hpp"

#include <cstddef>
#include <vector>

namespace racer_3d_cpp {

class VoxelMap;

struct SwarmPeer {
  int drone_id{-1};
  Point3 position{Point3::Zero()};
  Point3 velocity{Point3::Zero()};
};

struct AabbObstacle {
  Point3 minimum{Point3::Zero()};
  Point3 maximum{Point3::Zero()};
};

Point3 limitNorm(const Point3 &vector, double maximum);

Point3 cbfSwarmFilter(const Point3 &preferred, const Point3 &position,
                      const std::vector<SwarmPeer> &peers, double safe_distance,
                      double speed_limit, double alpha = 1.8,
                      int iterations = 3);

Point3 cbfSwarmFilter(const Point3 &preferred, const Point3 &position,
                      const std::vector<PeerState> &peers, double safe_distance,
                      double speed_limit, double alpha = 1.8,
                      int iterations = 3);

Point3 esdfObstacleFilter(const Point3 &preferred, const Point3 &position,
                          const VoxelMap &voxel_map, double clearance,
                          double speed_limit,
                          const Point3 &current_velocity = Point3::Zero(),
                          double alpha = 0.8,
                          double guaranteed_deceleration = 1.2,
                          double response_time = 0.12);

Point3 aabbObstacleFilter(const Point3 &preferred, const Point3 &position,
                          const std::vector<AabbObstacle> &obstacles,
                          double clearance, double speed_limit,
                          const Point3 &current_velocity = Point3::Zero(),
                          double alpha = 1.2,
                          double guaranteed_deceleration = 0.7,
                          double response_time = 0.08,
                          double activation_distance = 1.0, int iterations = 4);

Point3 pointcloudObstacleFilter(
    const Point3 &preferred, const Point3 &position,
    const std::vector<Point3> &points_world, double clearance,
    double speed_limit, const Point3 &current_velocity = Point3::Zero(),
    double alpha = 1.2, double guaranteed_deceleration = 0.7,
    double response_time = 0.10, double activation_distance = 1.0,
    std::size_t maximum_constraints = 64, int iterations = 4);

Point3 flightVolumeFilter(const Point3 &preferred, const Point3 &position,
                          const Point3 &minimum, const Point3 &maximum,
                          double clearance, double speed_limit,
                          const Point3 &current_velocity = Point3::Zero(),
                          double alpha = 1.2,
                          double guaranteed_deceleration = 0.7,
                          double response_time = 0.10, int iterations = 3);

Point3 emergencySeparation(const Point3 &velocity, const Point3 &position,
                           const std::vector<Point3> &peer_positions,
                           double emergency_distance, double speed_limit);

bool predictedPathConflict(const std::vector<TrajectorySample> &first,
                           const std::vector<TrajectorySample> &second,
                           double safe_distance, double time_tolerance = 0.35);

} // namespace racer_3d_cpp
