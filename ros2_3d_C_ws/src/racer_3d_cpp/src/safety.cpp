#include "racer_3d_cpp/safety.hpp"

#include "racer_3d_cpp/voxel_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace racer_3d_cpp {
namespace {

double stoppingAllowance(const Point3 &normal, const Point3 &current_velocity,
                         double guaranteed_deceleration, double response_time) {
  const double approach_speed = std::max(0.0, -normal.dot(current_velocity));
  return approach_speed * approach_speed /
             (2.0 * std::max(0.1, guaranteed_deceleration)) +
         response_time * approach_speed;
}

struct VelocityConstraint {
  double distance{};
  Point3 normal{Point3::Zero()};
  double lower_bound{};
};

Point3 projectConstraints(Point3 result,
                          std::vector<VelocityConstraint> constraints,
                          double speed_limit, int iterations) {
  std::sort(constraints.begin(), constraints.end(),
            [](const auto &first, const auto &second) {
              return first.distance < second.distance;
            });
  for (int iteration = 0; iteration < iterations; ++iteration) {
    for (const auto &constraint : constraints) {
      const double violation =
          constraint.lower_bound - constraint.normal.dot(result);
      if (violation > 0.0)
        result += violation * constraint.normal;
    }
    result = limitNorm(result, speed_limit);
  }
  return result;
}

} // namespace

Point3 limitNorm(const Point3 &vector, double maximum) {
  Point3 result = vector;
  const double norm = result.norm();
  if (norm > maximum && maximum > 0.0)
    result *= maximum / norm;
  return result;
}

Point3 cbfSwarmFilter(const Point3 &preferred, const Point3 &position,
                      const std::vector<SwarmPeer> &peers, double safe_distance,
                      double speed_limit, const Point3 &current_velocity,
                      double alpha, double guaranteed_deceleration,
                      double response_time, int iterations) {
  Point3 result = limitNorm(preferred, speed_limit);
  for (int iteration = 0; iteration < iterations; ++iteration) {
    for (const auto &peer : peers) {
      Point3 relative = position - peer.position;
      double distance = relative.norm();
      if (distance < 1.0e-6) {
        relative = Point3::UnitX();
        distance = 1.0e-6;
      }
      const Point3 normal = relative / distance;
      const Point3 relative_velocity = current_velocity - peer.velocity;
      const double approach_speed =
          std::max(0.0, -normal.dot(relative_velocity));
      const double stopping_allowance =
          approach_speed * approach_speed /
              (2.0 * std::max(0.1, guaranteed_deceleration)) +
          response_time * approach_speed;
      const double dynamic_safe_distance =
          safe_distance + stopping_allowance;
      const double h = distance * distance -
                       dynamic_safe_distance * dynamic_safe_distance;
      const double lower_bound =
          normal.dot(peer.velocity) - alpha * h / (2.0 * distance);
      const double violation = lower_bound - normal.dot(result);
      if (violation > 0.0)
        result += violation * normal;
    }
    result = limitNorm(result, speed_limit);
  }
  return result;
}

Point3 cbfSwarmFilter(const Point3 &preferred, const Point3 &position,
                      const std::vector<PeerState> &peers, double safe_distance,
                      double speed_limit, const Point3 &current_velocity,
                      double alpha, double guaranteed_deceleration,
                      double response_time, int iterations) {
  std::vector<SwarmPeer> values;
  values.reserve(peers.size());
  for (const auto &peer : peers) {
    values.push_back({peer.drone_id, peer.position, peer.velocity});
  }
  return cbfSwarmFilter(preferred, position, values, safe_distance, speed_limit,
                        current_velocity, alpha, guaranteed_deceleration,
                        response_time, iterations);
}

Point3 esdfObstacleFilter(const Point3 &preferred, const Point3 &position,
                          const VoxelMap &voxel_map, double clearance,
                          double speed_limit, const Point3 &current_velocity,
                          double alpha, double guaranteed_deceleration,
                          double response_time) {
  Point3 result = limitNorm(preferred, speed_limit);
  const double distance = voxel_map.distanceAt(position, false);
  if (!std::isfinite(distance))
    return Point3::Zero();

  const Point3 gradient = voxel_map.esdfGradient(position, false);
  const double norm = gradient.norm();
  if (norm < 1.0e-8)
    return result;
  const Point3 normal = gradient / norm;
  const double dynamic_clearance =
      clearance + stoppingAllowance(normal, current_velocity,
                                    guaranteed_deceleration, response_time);
  const double lower_bound = -alpha * (distance - dynamic_clearance);
  const double violation = lower_bound - normal.dot(result);
  if (violation > 0.0)
    result += violation * normal;
  return limitNorm(result, speed_limit);
}

Point3 aabbObstacleFilter(const Point3 &preferred, const Point3 &position,
                          const std::vector<AabbObstacle> &obstacles,
                          double clearance, double speed_limit,
                          const Point3 &current_velocity, double alpha,
                          double guaranteed_deceleration, double response_time,
                          double activation_distance, int iterations) {
  std::vector<VelocityConstraint> constraints;
  constraints.reserve(obstacles.size());
  for (const auto &obstacle : obstacles) {
    Point3 closest;
    for (int axis = 0; axis < 3; ++axis) {
      closest[axis] = std::clamp(position[axis], obstacle.minimum[axis],
                                 obstacle.maximum[axis]);
    }
    Point3 delta = position - closest;
    double signed_distance = delta.norm();
    Point3 normal = Point3::Zero();
    if (signed_distance > 1.0e-9) {
      normal = delta / signed_distance;
    } else {
      double closest_face = std::numeric_limits<double>::infinity();
      int face_axis = 0;
      double direction = 1.0;
      for (int axis = 0; axis < 3; ++axis) {
        const double to_minimum = position[axis] - obstacle.minimum[axis];
        if (to_minimum < closest_face) {
          closest_face = to_minimum;
          face_axis = axis;
          direction = -1.0;
        }
        const double to_maximum = obstacle.maximum[axis] - position[axis];
        if (to_maximum < closest_face) {
          closest_face = to_maximum;
          face_axis = axis;
          direction = 1.0;
        }
      }
      normal[face_axis] = direction;
      signed_distance = -closest_face;
    }
    if (signed_distance > activation_distance)
      continue;
    const double dynamic_clearance =
        clearance + stoppingAllowance(normal, current_velocity,
                                      guaranteed_deceleration, response_time);
    constraints.push_back({signed_distance, normal,
                           -alpha * (signed_distance - dynamic_clearance)});
  }
  return projectConstraints(limitNorm(preferred, speed_limit),
                            std::move(constraints), speed_limit, iterations);
}

Point3 pointcloudObstacleFilter(const Point3 &preferred, const Point3 &position,
                                const std::vector<Point3> &points_world,
                                double clearance, double speed_limit,
                                const Point3 &current_velocity, double alpha,
                                double guaranteed_deceleration,
                                double response_time,
                                double activation_distance,
                                std::size_t maximum_constraints,
                                int iterations) {
  struct PointDistance {
    double distance{};
    Point3 delta{Point3::Zero()};
  };
  std::vector<PointDistance> nearby;
  nearby.reserve(points_world.size());
  for (const auto &point : points_world) {
    if (!point.allFinite())
      continue;
    const Point3 delta = position - point;
    const double distance = delta.norm();
    if (distance > 1.0e-4 && distance <= activation_distance) {
      nearby.push_back({distance, delta});
    }
  }
  std::sort(nearby.begin(), nearby.end(),
            [](const auto &first, const auto &second) {
              return first.distance < second.distance;
            });
  if (nearby.size() > maximum_constraints) {
    nearby.resize(maximum_constraints);
  }

  std::vector<VelocityConstraint> constraints;
  constraints.reserve(nearby.size());
  for (const auto &item : nearby) {
    const Point3 normal = item.delta / item.distance;
    const double dynamic_clearance =
        clearance + stoppingAllowance(normal, current_velocity,
                                      guaranteed_deceleration, response_time);
    constraints.push_back(
        {item.distance, normal, -alpha * (item.distance - dynamic_clearance)});
  }
  return projectConstraints(limitNorm(preferred, speed_limit),
                            std::move(constraints), speed_limit, iterations);
}

Point3 flightVolumeFilter(const Point3 &preferred, const Point3 &position,
                          const Point3 &minimum, const Point3 &maximum,
                          double clearance, double speed_limit,
                          const Point3 &current_velocity, double alpha,
                          double guaranteed_deceleration, double response_time,
                          int iterations) {
  std::vector<VelocityConstraint> constraints;
  constraints.reserve(6);
  for (int axis = 0; axis < 3; ++axis) {
    for (const auto &[distance, direction] :
         std::array<std::pair<double, double>, 2>{
             std::make_pair(position[axis] - minimum[axis], 1.0),
             std::make_pair(maximum[axis] - position[axis], -1.0)}) {
      Point3 normal = Point3::Zero();
      normal[axis] = direction;
      const double dynamic_clearance =
          clearance + stoppingAllowance(normal, current_velocity,
                                        guaranteed_deceleration, response_time);
      constraints.push_back(
          {distance, normal, -alpha * (distance - dynamic_clearance)});
    }
  }
  return projectConstraints(limitNorm(preferred, speed_limit),
                            std::move(constraints), speed_limit, iterations);
}

Point3 emergencySeparation(const Point3 &velocity, const Point3 &position,
                           const std::vector<Point3> &peer_positions,
                           double emergency_distance, double speed_limit) {
  Point3 result = velocity;
  for (const auto &peer : peer_positions) {
    Point3 delta = position - peer;
    double distance = delta.norm();
    if (distance >= emergency_distance)
      continue;
    if (distance < 1.0e-6) {
      delta = Point3::UnitX();
      distance = 1.0e-6;
    }
    const double strength = speed_limit * (emergency_distance - distance) /
                            std::max(emergency_distance, 1.0e-6);
    result += strength * delta / distance;
  }
  return limitNorm(result, speed_limit);
}

bool predictedPathConflict(const std::vector<TrajectorySample> &first,
                           const std::vector<TrajectorySample> &second,
                           double safe_distance, double time_tolerance) {
  for (const auto &own : first) {
    for (const auto &peer : second) {
      if (std::abs(own.time - peer.time) > time_tolerance)
        continue;
      if ((own.position - peer.position).norm() < safe_distance)
        return true;
    }
  }
  return false;
}

} // namespace racer_3d_cpp
