#include "racer_3d_cpp/planning.hpp"

#include "racer_3d_cpp/hgrid.hpp"
#include "racer_3d_cpp/voxel_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace racer_3d_cpp {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::size_t flatIndex(const GridIndex3 &cell, int nx, int ny) {
  return static_cast<std::size_t>(cell.x) +
         static_cast<std::size_t>(nx) *
             (static_cast<std::size_t>(cell.y) +
              static_cast<std::size_t>(ny) * static_cast<std::size_t>(cell.z));
}

bool validCell(const GridIndex3 &cell, int nx, int ny, int nz) {
  return cell.x >= 0 && cell.x < nx && cell.y >= 0 && cell.y < ny &&
         cell.z >= 0 && cell.z < nz;
}

bool diagonalIsSafe(const GridIndex3 &current, int dx, int dy, int dz,
                    const std::vector<std::uint8_t> &blocked, int nx, int ny,
                    int nz) {
  const std::array<int, 3> delta{dx, dy, dz};
  std::array<int, 3> changed{};
  int changed_count = 0;
  for (int axis = 0; axis < 3; ++axis) {
    if (delta[axis] != 0)
      changed[changed_count++] = axis;
  }
  if (changed_count <= 1)
    return true;

  // Every proper non-empty subset of a diagonal move must remain free.
  const int full_mask = (1 << changed_count) - 1;
  for (int mask = 1; mask < full_mask; ++mask) {
    GridIndex3 intermediate = current;
    for (int bit = 0; bit < changed_count; ++bit) {
      if ((mask & (1 << bit)) == 0)
        continue;
      switch (changed[bit]) {
      case 0:
        intermediate.x += dx;
        break;
      case 1:
        intermediate.y += dy;
        break;
      default:
        intermediate.z += dz;
        break;
      }
    }
    if (!validCell(intermediate, nx, ny, nz) ||
        blocked[flatIndex(intermediate, nx, ny)] != 0) {
      return false;
    }
  }
  return true;
}

bool lineFree(const VoxelMap &voxel_map, const GridIndex3 &first,
              const GridIndex3 &second,
              const std::vector<std::uint8_t> &blocked) {
  for (const auto &cell : voxel_map.rayCells(voxel_map.gridToWorld(first),
                                             voxel_map.gridToWorld(second))) {
    if (blocked[voxel_map.flatIndex(cell)] != 0)
      return false;
  }
  return true;
}

std::vector<TrajectorySample> samplePolyline(const std::vector<Point3> &points,
                                             double max_speed,
                                             double sample_dt) {
  std::vector<TrajectorySample> result;
  if (points.empty())
    return result;
  result.push_back({0.0, points.front()});
  double timestamp = 0.0;
  for (std::size_t index = 0; index + 1 < points.size(); ++index) {
    const Point3 &first = points[index];
    const Point3 &second = points[index + 1];
    const double distance = (second - first).norm();
    const double duration =
        std::max(sample_dt, distance / std::max(0.05, max_speed));
    const int count =
        std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    for (int sample = 1; sample <= count; ++sample) {
      const double ratio =
          static_cast<double>(sample) / static_cast<double>(count);
      timestamp += duration / static_cast<double>(count);
      result.push_back({timestamp, first + ratio * (second - first)});
    }
  }
  return result;
}

std::vector<Point3> viewpointCandidates(
    const VoxelMap &voxel_map, const std::vector<GridIndex3> &cluster,
    const std::vector<std::uint8_t> &blocked, double clearance) {
  if (cluster.empty())
    return {};
  Point3 centroid = Point3::Zero();
  for (const auto &cell : cluster)
    centroid += voxel_map.gridToWorld(cell);
  centroid /= static_cast<double>(cluster.size());

  std::vector<Point3> candidates;
  const std::array<double, 3> elevations{-35.0 * kPi / 180.0, 0.0,
                                         35.0 * kPi / 180.0};
  for (const double radius : {0.9, 1.45}) {
    for (const double elevation : elevations) {
      const double horizontal = radius * std::cos(elevation);
      for (int sample = 0; sample < 8; ++sample) {
        const double azimuth =
            -kPi + 2.0 * kPi * static_cast<double>(sample) / 8.0;
        const Point3 point = centroid + Point3(horizontal * std::cos(azimuth),
                                               horizontal * std::sin(azimuth),
                                               radius * std::sin(elevation));
        const auto cell = voxel_map.worldToGrid(point);
        if (cell && blocked[voxel_map.flatIndex(*cell)] == 0 &&
            voxel_map.distanceAt(point, false) >= clearance) {
          candidates.push_back(point);
        }
      }
    }
  }
  return candidates;
}

struct RankedPlanCandidate {
  double score{-std::numeric_limits<double>::infinity()};
  int gain{};
  std::vector<GridIndex3> path;
  Point3 viewpoint{Point3::Zero()};
  Point3 centroid{Point3::Zero()};
};

} // namespace

std::vector<GridIndex3> astar3d(const std::vector<std::uint8_t> &blocked,
                                int nx, int ny, int nz, const GridIndex3 &start,
                                const GridIndex3 &goal, double resolution) {
  const std::size_t expected = static_cast<std::size_t>(std::max(0, nx)) *
                               static_cast<std::size_t>(std::max(0, ny)) *
                               static_cast<std::size_t>(std::max(0, nz));
  if (blocked.size() != expected || !validCell(start, nx, ny, nz) ||
      !validCell(goal, nx, ny, nz)) {
    return {};
  }
  std::vector<std::uint8_t> effective = blocked;
  effective[flatIndex(start, nx, ny)] = 0;
  if (effective[flatIndex(goal, nx, ny)] != 0)
    return {};

  struct QueueEntry {
    double priority{};
    double cost{};
    GridIndex3 cell;
  };
  const auto compare = [](const QueueEntry &first, const QueueEntry &second) {
    if (first.priority != second.priority) {
      return first.priority > second.priority;
    }
    return first.cost > second.cost;
  };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, decltype(compare)>
      queue(compare);
  queue.push({0.0, 0.0, start});

  std::unordered_map<GridIndex3, GridIndex3, GridIndex3Hash> came_from;
  std::unordered_map<GridIndex3, double, GridIndex3Hash> cost;
  cost[start] = 0.0;

  while (!queue.empty()) {
    const QueueEntry entry = queue.top();
    queue.pop();
    const GridIndex3 current = entry.cell;
    if (current == goal) {
      std::vector<GridIndex3> result{current};
      auto iterator = came_from.find(current);
      GridIndex3 cursor = current;
      while (iterator != came_from.end()) {
        cursor = iterator->second;
        result.push_back(cursor);
        iterator = came_from.find(cursor);
      }
      std::reverse(result.begin(), result.end());
      return result;
    }
    const auto current_cost = cost.find(current);
    if (current_cost == cost.end() ||
        entry.cost > current_cost->second + 1.0e-9) {
      continue;
    }

    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0)
            continue;
          const GridIndex3 neighbor{current.x + dx, current.y + dy,
                                    current.z + dz};
          if (!validCell(neighbor, nx, ny, nz) ||
              effective[flatIndex(neighbor, nx, ny)] != 0 ||
              !diagonalIsSafe(current, dx, dy, dz, effective, nx, ny, nz)) {
            continue;
          }
          const double step =
              resolution *
              std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
          const double candidate = entry.cost + step;
          const auto known = cost.find(neighbor);
          if (known != cost.end() && candidate + 1.0e-9 >= known->second) {
            continue;
          }
          cost[neighbor] = candidate;
          came_from[neighbor] = current;
          const double gx = static_cast<double>(goal.x - neighbor.x);
          const double gy = static_cast<double>(goal.y - neighbor.y);
          const double gz = static_cast<double>(goal.z - neighbor.z);
          const double heuristic =
              resolution * std::sqrt(gx * gx + gy * gy + gz * gz);
          queue.push({candidate + heuristic, candidate, neighbor});
        }
      }
    }
  }
  return {};
}

std::vector<GridIndex3>
shortenPath3d(const VoxelMap &voxel_map, const std::vector<GridIndex3> &path,
              const std::vector<std::uint8_t> &blocked) {
  if (path.size() <= 2)
    return path;
  std::vector<GridIndex3> result{path.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    std::size_t candidate = path.size() - 1;
    while (candidate > anchor + 1 &&
           !lineFree(voxel_map, path[anchor], path[candidate], blocked)) {
      --candidate;
    }
    result.push_back(path[candidate]);
    anchor = candidate;
  }
  return result;
}

UniformBSpline3D::UniformBSpline3D(const std::vector<Point3> &control_points,
                                   int degree)
    : points_(control_points) {
  if (points_.empty()) {
    throw std::invalid_argument("B-spline requires at least one control point");
  }
  const int requested_degree = std::max(0, degree);
  while (points_.size() < static_cast<std::size_t>(requested_degree + 1)) {
    const Point3 duplicate = points_.back();
    if (points_.size() > 1) {
      points_.insert(points_.end() - 1, duplicate);
    } else {
      points_.push_back(duplicate);
    }
  }
  degree_ = std::min(requested_degree, static_cast<int>(points_.size()) - 1);
  knots_.assign(static_cast<std::size_t>(degree_ + 1), 0.0);
  const int interior = static_cast<int>(points_.size()) - degree_ - 1;
  for (int index = 1; index <= interior; ++index) {
    knots_.push_back(static_cast<double>(index) /
                     static_cast<double>(interior + 1));
  }
  knots_.insert(knots_.end(), static_cast<std::size_t>(degree_ + 1), 1.0);
}

Point3 UniformBSpline3D::evaluate(double parameter) const {
  const double u = std::clamp(parameter, 0.0, 1.0);
  const int final = static_cast<int>(points_.size()) - 1;
  if (u >= 1.0)
    return points_.back();
  int span = degree_;
  while (span < final && !(knots_[span] <= u && u < knots_[span + 1])) {
    ++span;
  }
  std::vector<Point3> work;
  work.reserve(static_cast<std::size_t>(degree_ + 1));
  for (int index = 0; index <= degree_; ++index) {
    work.push_back(points_[span - degree_ + index]);
  }
  for (int level = 1; level <= degree_; ++level) {
    for (int index = degree_; index >= level; --index) {
      const int knot_index = span - degree_ + index;
      const double denominator =
          knots_[knot_index + degree_ - level + 1] - knots_[knot_index];
      const double alpha =
          denominator == 0.0 ? 0.0 : (u - knots_[knot_index]) / denominator;
      work[index] = (1.0 - alpha) * work[index - 1] + alpha * work[index];
    }
  }
  return work[degree_];
}

std::vector<Point3>
optimizeBsplineControlPoints(const std::vector<Point3> &points,
                             const VoxelMap &voxel_map, double clearance,
                             int iterations) {
  std::vector<Point3> control = points;
  if (control.size() < 3)
    return control;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    std::vector<Point3> updated = control;
    const double rate =
        0.20 * (1.0 - 0.6 * static_cast<double>(iteration) /
                          static_cast<double>(std::max(1, iterations - 1)));
    for (std::size_t index = 1; index + 1 < control.size(); ++index) {
      const Point3 smooth_gradient =
          control[index - 1] - 2.0 * control[index] + control[index + 1];
      const double distance = voxel_map.distanceAt(control[index], false);
      Point3 obstacle_gradient = Point3::Zero();
      if (distance < 1.35 * clearance) {
        const Point3 direction = voxel_map.esdfGradient(control[index], false);
        const double norm = direction.norm();
        if (norm > 1.0e-6) {
          obstacle_gradient = direction / norm * (1.35 * clearance - distance);
        }
      }
      updated[index] +=
          rate * (0.30 * smooth_gradient + 1.8 * obstacle_gradient);
    }
    control = std::move(updated);
  }
  return control;
}

std::vector<Point3>
optimizeBSplineControlPoints(const std::vector<Point3> &points,
                             const VoxelMap &voxel_map, double clearance,
                             int iterations) {
  return optimizeBsplineControlPoints(points, voxel_map, clearance, iterations);
}

TrajectoryResult minimumTimeBsplineTrajectory(const std::vector<Point3> &points,
                                              const VoxelMap &voxel_map,
                                              double clearance,
                                              double max_speed,
                                              double max_acceleration,
                                              double sample_dt) {
  if (points.empty()) {
    return {{}, -std::numeric_limits<double>::infinity()};
  }
  if (points.size() == 1) {
    return {{{0.0, points.front()}}, std::numeric_limits<double>::infinity()};
  }
  const std::vector<Point3> optimized =
      optimizeBsplineControlPoints(points, voxel_map, clearance);
  const UniformBSpline3D spline(optimized);
  std::vector<Point3> dense;
  dense.reserve(201);
  for (int index = 0; index <= 200; ++index) {
    dense.push_back(spline.evaluate(static_cast<double>(index) / 200.0));
  }

  double minimum_clearance = std::numeric_limits<double>::infinity();
  bool spline_known = true;
  for (const auto &point : dense) {
    minimum_clearance =
        std::min(minimum_clearance, voxel_map.distanceAt(point, false));
    const auto cell = voxel_map.worldToGrid(point);
    if (!cell || voxel_map.stateAt(*cell) != FREE)
      spline_known = false;
  }
  if (minimum_clearance < clearance || !spline_known) {
    auto trajectory = samplePolyline(points, max_speed, sample_dt);
    double polyline_clearance = std::numeric_limits<double>::infinity();
    for (const auto &sample : trajectory) {
      polyline_clearance = std::min(
          polyline_clearance, voxel_map.distanceAt(sample.position, false));
    }
    return {std::move(trajectory), polyline_clearance};
  }

  double length = 0.0;
  for (std::size_t index = 0; index + 1 < dense.size(); ++index) {
    length += (dense[index + 1] - dense[index]).norm();
  }
  const double speed = std::max(0.05, max_speed);
  const double acceleration = std::max(0.05, max_acceleration);
  const double acceleration_time = speed / acceleration;
  const double acceleration_distance =
      acceleration * acceleration_time * acceleration_time;
  double duration = 0.0;
  if (length <= acceleration_distance) {
    duration = 2.0 * std::sqrt(length / acceleration);
  } else {
    duration = acceleration_time + length / speed;
  }
  duration = std::max(sample_dt, duration);
  const int count =
      std::max(2, static_cast<int>(std::ceil(duration / sample_dt)));
  std::vector<TrajectorySample> trajectory;
  trajectory.reserve(static_cast<std::size_t>(count + 1));
  for (int index = 0; index <= count; ++index) {
    const double ratio =
        static_cast<double>(index) / static_cast<double>(count);
    trajectory.push_back({duration * ratio, spline.evaluate(ratio)});
  }
  return {std::move(trajectory), minimum_clearance};
}

std::pair<std::vector<TrajectorySample>, double>
minimumTimeBSplineTrajectory(const std::vector<Point3> &points,
                             const VoxelMap &voxel_map, double clearance,
                             double max_speed, double max_acceleration,
                             double sample_dt) {
  auto result = minimumTimeBsplineTrajectory(
      points, voxel_map, clearance, max_speed, max_acceleration, sample_dt);
  return {std::move(result.samples), result.minimum_clearance};
}

std::optional<ExplorationPlan3D>
planExploration(const VoxelMap &voxel_map, const HierarchicalGrid3D &hgrid,
                const std::vector<std::string> &owned_cells,
                const std::vector<std::string> &coverage_route,
                const Point3 &position, double clearance, double max_speed,
                double max_acceleration) {
  const auto start = voxel_map.worldToGrid(position);
  if (!start)
    return std::nullopt;

  const double search_clearance =
      clearance + 0.5 * std::sqrt(3.0) * voxel_map.resolution();
  const auto state = voxel_map.states();
  auto blocked = voxel_map.inflatedBlocked(search_clearance, false);
  if (blocked.size() != state.size())
    return std::nullopt;
  for (std::size_t index = 0; index < blocked.size(); ++index) {
    if (state[index] != FREE)
      blocked[index] = 1;
  }
  blocked[voxel_map.flatIndex(*start)] = 0;

  const auto clusters = voxel_map.frontierClusters();
  if (clusters.empty())
    return std::nullopt;

  std::unordered_map<std::string, int> route_rank;
  for (std::size_t index = 0; index < coverage_route.size(); ++index) {
    route_rank[coverage_route[index]] = static_cast<int>(index);
  }
  const std::unordered_set<std::string> owned(owned_cells.begin(),
                                              owned_cells.end());

  std::vector<std::vector<GridIndex3>> segments;
  for (const auto &cluster : clusters) {
    std::map<std::string, std::vector<GridIndex3>> grouped;
    for (const auto &cell : cluster) {
      const auto hcell = hgrid.containing(voxel_map.gridToWorld(cell));
      grouped[hcell ? hcell->id() : std::string{}].push_back(cell);
    }
    std::vector<std::vector<GridIndex3>> retained;
    for (auto &[id, values] : grouped) {
      (void)id;
      if (values.size() >= 4)
        retained.push_back(std::move(values));
    }
    if (retained.empty()) {
      segments.push_back(cluster);
    } else {
      segments.insert(segments.end(), std::make_move_iterator(retained.begin()),
                      std::make_move_iterator(retained.end()));
    }
  }

  std::vector<std::vector<GridIndex3>> owned_segments;
  for (const auto &segment : segments) {
    Point3 centroid = Point3::Zero();
    for (const auto &cell : segment) {
      centroid += voxel_map.gridToWorld(cell);
    }
    centroid /= static_cast<double>(segment.size());
    const auto hcell = hgrid.containing(centroid);
    if (hcell && owned.count(hcell->id()) != 0) {
      owned_segments.push_back(segment);
    }
  }

  const auto bestCandidate =
      [&](const std::vector<std::vector<GridIndex3>> &candidate_segments)
      -> std::optional<RankedPlanCandidate> {
    std::vector<const std::vector<GridIndex3> *> ordered;
    ordered.reserve(candidate_segments.size());
    for (const auto &segment : candidate_segments) {
      ordered.push_back(&segment);
    }
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const auto *first, const auto *second) {
                       return first->size() > second->size();
                     });
    if (ordered.size() > 12)
      ordered.resize(12);

    std::optional<RankedPlanCandidate> best;
    for (const auto *cluster : ordered) {
      Point3 centroid = Point3::Zero();
      for (const auto &cell : *cluster) {
        centroid += voxel_map.gridToWorld(cell);
      }
      centroid /= static_cast<double>(cluster->size());
      const auto hcell = hgrid.containing(centroid);
      double owner_bonus = 0.0;
      if (hcell && owned.count(hcell->id()) != 0) {
        const auto rank = route_rank.find(hcell->id());
        owner_bonus = 12.0 - static_cast<double>(
                                 rank == route_rank.end() ? 8 : rank->second);
      }

      struct RankedViewpoint {
        double score{};
        Point3 point{Point3::Zero()};
      };
      std::vector<RankedViewpoint> viewpoints;
      for (const auto &point : viewpointCandidates(voxel_map, *cluster, blocked,
                                                   search_clearance)) {
        viewpoints.push_back({static_cast<double>(voxel_map.visibleUnknownGain(
                                  point, *cluster)) -
                                  0.8 * (point - position).norm(),
                              point});
      }
      std::sort(viewpoints.begin(), viewpoints.end(),
                [](const auto &first, const auto &second) {
                  return first.score > second.score;
                });
      if (viewpoints.size() > 2)
        viewpoints.resize(2);

      for (const auto &candidate_view : viewpoints) {
        const auto goal = voxel_map.worldToGrid(candidate_view.point);
        if (!goal)
          continue;
        auto path =
            astar3d(blocked, voxel_map.nx(), voxel_map.ny(), voxel_map.nz(),
                    *start, *goal, voxel_map.resolution());
        if (path.empty())
          continue;

        double path_distance = 0.0;
        for (std::size_t index = 0; index + 1 < path.size(); ++index) {
          const double dx =
              static_cast<double>(path[index + 1].x - path[index].x);
          const double dy =
              static_cast<double>(path[index + 1].y - path[index].y);
          const double dz =
              static_cast<double>(path[index + 1].z - path[index].z);
          path_distance +=
              voxel_map.resolution() * std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        const int gain =
            voxel_map.visibleUnknownGain(candidate_view.point, *cluster);
        RankedPlanCandidate candidate;
        candidate.score = static_cast<double>(gain) +
                          0.12 * static_cast<double>(cluster->size()) +
                          owner_bonus - 1.4 * path_distance;
        candidate.gain = gain;
        candidate.path = std::move(path);
        candidate.viewpoint = candidate_view.point;
        candidate.centroid = centroid;
        if (!best || candidate.score > best->score) {
          best = std::move(candidate);
        }
      }
    }
    return best;
  };

  std::optional<RankedPlanCandidate> best =
      bestCandidate(owned_segments.empty() ? segments : owned_segments);
  if (!best && !owned_segments.empty())
    best = bestCandidate(segments);
  if (!best)
    return std::nullopt;

  const auto shortened = shortenPath3d(voxel_map, best->path, blocked);
  std::vector<Point3> points{position};
  for (std::size_t index = 1; index + 1 < shortened.size(); ++index) {
    points.push_back(voxel_map.gridToWorld(shortened[index]));
  }
  points.push_back(best->viewpoint);
  TrajectoryResult trajectory = minimumTimeBsplineTrajectory(
      points, voxel_map, clearance, max_speed, max_acceleration);
  const Point3 direction = best->centroid - best->viewpoint;

  ExplorationPlan3D plan;
  plan.goal = best->viewpoint;
  plan.yaw = std::atan2(direction.y(), direction.x());
  plan.pitch =
      std::atan2(direction.z(),
                 std::max(1.0e-6, std::hypot(direction.x(), direction.y())));
  plan.frontier_centroid = best->centroid;
  plan.grid_path = std::move(best->path);
  plan.geometric_path = std::move(points);
  plan.trajectory = std::move(trajectory.samples);
  plan.information_gain = best->gain;
  plan.minimum_clearance = trajectory.minimum_clearance;
  return plan;
}

} // namespace racer_3d_cpp
