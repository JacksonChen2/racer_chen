#include "racer_3d_cpp/hgrid.hpp"

#include "racer_3d_cpp/voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

namespace racer_3d_cpp {
namespace {

double distance(const Point3 &first, const Point3 &second,
                const HGridDistanceFunction &distance_function) {
  if (distance_function) {
    const double value = distance_function(first, second);
    if (std::isfinite(value)) {
      return value;
    }
  }
  return (second - first).norm();
}

}  // namespace

std::string HGridCell3D::id() const {
  return std::to_string(level) + ":" + std::to_string(ix) + ":" +
         std::to_string(iy) + ":" + std::to_string(iz);
}

HierarchicalGrid3D::HierarchicalGrid3D(
    const VoxelMap &voxel_map, const Point3 &coarse_size, int levels,
    double subdivide_known_ratio, int minimum_unknown)
    : map_(voxel_map),
      coarse_size_(coarse_size),
      levels_(levels),
      subdivide_known_ratio_(subdivide_known_ratio),
      minimum_unknown_(minimum_unknown) {
  if (levels_ < 1) {
    throw std::invalid_argument("hgrid needs at least one level");
  }
  if ((coarse_size_.array() <= 0.0).any()) {
    throw std::invalid_argument("hgrid coarse size must be positive");
  }
  if (!std::isfinite(subdivide_known_ratio_) ||
      subdivide_known_ratio_ < 0.0 || subdivide_known_ratio_ > 1.0) {
    throw std::invalid_argument(
        "hgrid subdivision known ratio must be in [0, 1]");
  }
  if (minimum_unknown_ < 0) {
    throw std::invalid_argument("hgrid minimum unknown must be nonnegative");
  }
}

std::array<int, 3> HierarchicalGrid3D::span(int level) const {
  const double factor = std::pow(2.0, static_cast<double>(level - 1));
  const double resolution = map_.resolution();
  if (!(resolution > 0.0) || !std::isfinite(resolution)) {
    throw std::runtime_error("voxel map resolution must be positive");
  }
  std::array<int, 3> result{};
  for (int axis = 0; axis < 3; ++axis) {
    // nearbyint follows the default ties-to-even mode, matching Python round.
    result[axis] = std::max(
        1, static_cast<int>(
               std::nearbyint(coarse_size_[axis] / factor / resolution)));
  }
  return result;
}

HGridCell3D HierarchicalGrid3D::cellFromIndex(int level, int ix, int iy,
                                               int iz) const {
  const auto cell_span = span(level);
  const int x0 = ix * cell_span[0];
  const int y0 = iy * cell_span[1];
  const int z0 = iz * cell_span[2];
  const int x1 = std::min(map_.nx(), x0 + cell_span[0]);
  const int y1 = std::min(map_.ny(), y0 + cell_span[1]);
  const int z1 = std::min(map_.nz(), z0 + cell_span[2]);

  const auto &states = map_.states();
  const std::size_t expected_size =
      static_cast<std::size_t>(map_.nx()) *
      static_cast<std::size_t>(map_.ny()) *
      static_cast<std::size_t>(map_.nz());
  if (states.size() != expected_size) {
    throw std::runtime_error("voxel state size does not match dimensions");
  }

  int unknown = 0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  for (int z = z0; z < z1; ++z) {
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        const std::size_t flat =
            (static_cast<std::size_t>(z) *
                 static_cast<std::size_t>(map_.ny()) +
             static_cast<std::size_t>(y)) *
                static_cast<std::size_t>(map_.nx()) +
            static_cast<std::size_t>(x);
        if (states[flat] == UNKNOWN) {
          ++unknown;
          sum_x += static_cast<double>(x);
          sum_y += static_cast<double>(y);
          sum_z += static_cast<double>(z);
        }
      }
    }
  }

  GridIndex3 centroid_index{};
  if (unknown > 0) {
    centroid_index = {
        static_cast<int>(std::nearbyint(sum_x / unknown)),
        static_cast<int>(std::nearbyint(sum_y / unknown)),
        static_cast<int>(std::nearbyint(sum_z / unknown)),
    };
  } else {
    centroid_index = {
        std::min(map_.nx() - 1, (x0 + x1) / 2),
        std::min(map_.ny() - 1, (y0 + y1) / 2),
        std::min(map_.nz() - 1, (z0 + z1) / 2),
    };
  }

  const int size_x = std::max(0, x1 - x0);
  const int size_y = std::max(0, y1 - y0);
  const int size_z = std::max(0, z1 - z0);
  return HGridCell3D{
      level,
      ix,
      iy,
      iz,
      {x0, y0, z0, x1, y1, z1},
      map_.gridToWorld(centroid_index),
      unknown,
      std::max(1, size_x * size_y * size_z),
  };
}

const HierarchicalGrid3D::CellMap &HierarchicalGrid3D::update() {
  CellMap active;
  std::vector<std::string> active_order;

  std::function<void(int, int, int, int)> visit =
      [&](int level, int ix, int iy, int iz) {
        HGridCell3D cell = cellFromIndex(level, ix, iy, iz);
        const int x0 = cell.bounds[0];
        const int y0 = cell.bounds[1];
        const int z0 = cell.bounds[2];
        const int x1 = cell.bounds[3];
        const int y1 = cell.bounds[4];
        const int z1 = cell.bounds[5];
        if (x0 >= x1 || y0 >= y1 || z0 >= z1) {
          return;
        }
        const double known_ratio =
            1.0 - static_cast<double>(cell.unknownCount) /
                      static_cast<double>(cell.total);
        if (level < levels_ &&
            known_ratio >= subdivide_known_ratio_ &&
            cell.unknownCount >= minimum_unknown_) {
          for (int dz = 0; dz <= 1; ++dz) {
            for (int dy = 0; dy <= 1; ++dy) {
              for (int dx = 0; dx <= 1; ++dx) {
                visit(level + 1, ix * 2 + dx, iy * 2 + dy,
                      iz * 2 + dz);
              }
            }
          }
          return;
        }
        if (cell.unknownCount >= minimum_unknown_) {
          const std::string cell_id = cell.id();
          active_order.push_back(cell_id);
          active.emplace(cell_id, std::move(cell));
        }
      };

  const auto coarse_span = span(1);
  const int count_x = (map_.nx() + coarse_span[0] - 1) / coarse_span[0];
  const int count_y = (map_.ny() + coarse_span[1] - 1) / coarse_span[1];
  const int count_z = (map_.nz() + coarse_span[2] - 1) / coarse_span[2];
  for (int iz = 0; iz < count_z; ++iz) {
    for (int iy = 0; iy < count_y; ++iy) {
      for (int ix = 0; ix < count_x; ++ix) {
        visit(1, ix, iy, iz);
      }
    }
  }
  cells_ = std::move(active);
  cell_order_ = std::move(active_order);
  return cells_;
}

std::optional<HGridCell3D> HierarchicalGrid3D::containing(
    const Point3 &point) const {
  std::vector<HGridCell3D> candidates;
  candidates.reserve(cell_order_.size());
  for (const auto &cell_id : cell_order_) {
    const auto found = cells_.find(cell_id);
    if (found != cells_.end()) {
      candidates.push_back(found->second);
    }
  }
  return containing(point, candidates);
}

std::optional<HGridCell3D> HierarchicalGrid3D::containing(
    const Point3 &point,
    const std::vector<HGridCell3D> &candidate_cells) const {
  const auto index = map_.worldToGrid(point);
  if (!index) {
    return std::nullopt;
  }
  std::vector<HGridCell3D> candidates = candidate_cells;
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const HGridCell3D &first,
                      const HGridCell3D &second) {
                     return first.level > second.level;
                   });
  for (const auto &cell : candidates) {
    if (cell.bounds[0] <= index->x && index->x < cell.bounds[3] &&
        cell.bounds[1] <= index->y && index->y < cell.bounds[4] &&
        cell.bounds[2] <= index->z && index->z < cell.bounds[5]) {
      return cell;
    }
  }
  return std::nullopt;
}

std::map<std::string, int> HierarchicalGrid3D::initialOwners(
    const std::vector<Point3> &starts) const {
  if (starts.empty()) {
    throw std::invalid_argument("initialOwners requires at least one start");
  }
  std::vector<HGridCell3D> active;
  active.reserve(cell_order_.size());
  for (const auto &cell_id : cell_order_) {
    const auto found = cells_.find(cell_id);
    if (found != cells_.end()) {
      active.push_back(found->second);
    }
  }
  std::stable_sort(active.begin(), active.end(),
                   [](const HGridCell3D &first,
                      const HGridCell3D &second) {
                     return first.demand() > second.demand();
                   });

  std::map<std::string, int> owners;
  std::vector<int> load(starts.size(), 0);
  for (const auto &cell : active) {
    int owner = 0;
    double best_score =
        (cell.centroid - starts.front()).norm() + 0.004 * load.front();
    for (std::size_t drone_id = 1; drone_id < starts.size(); ++drone_id) {
      const double score = (cell.centroid - starts[drone_id]).norm() +
                           0.004 * load[drone_id];
      if (score < best_score) {
        owner = static_cast<int>(drone_id);
        best_score = score;
      }
    }
    owners[cell.id()] = owner;
    load[static_cast<std::size_t>(owner)] += cell.demand();
  }
  return owners;
}

std::vector<std::string> HierarchicalGrid3D::coverageRoute(
    const std::vector<std::string> &cell_ids, const Point3 &start,
    const HGridDistanceFunction &distance_function) const {
  std::vector<HGridCell3D> remaining;
  remaining.reserve(cell_ids.size());
  for (const auto &cell_id : cell_ids) {
    const auto found = cells_.find(cell_id);
    if (found != cells_.end()) {
      remaining.push_back(found->second);
    }
  }

  std::vector<HGridCell3D> route;
  route.reserve(remaining.size());
  Point3 cursor = start;
  while (!remaining.empty()) {
    std::size_t selected = 0;
    double selected_distance =
        distance(cursor, remaining.front().centroid, distance_function);
    for (std::size_t index = 1; index < remaining.size(); ++index) {
      const double candidate_distance =
          distance(cursor, remaining[index].centroid, distance_function);
      if (candidate_distance < selected_distance) {
        selected = index;
        selected_distance = candidate_distance;
      }
    }
    route.push_back(remaining[selected]);
    cursor = remaining[selected].centroid;
    remaining.erase(remaining.begin() +
                    static_cast<std::ptrdiff_t>(selected));
  }
  if (route.size() > 3) {
    route = twoOpt(route, start, distance_function);
  }

  std::vector<std::string> result;
  result.reserve(route.size());
  for (const auto &cell : route) {
    result.push_back(cell.id());
  }
  return result;
}

std::vector<HGridCell3D> HierarchicalGrid3D::twoOpt(
    const std::vector<HGridCell3D> &route, const Point3 &start,
    const HGridDistanceFunction &distance_function) const {
  const auto cost = [&](const std::vector<HGridCell3D> &items) {
    Point3 cursor = start;
    double result = 0.0;
    for (const auto &item : items) {
      result += distance(cursor, item.centroid, distance_function);
      cursor = item.centroid;
    }
    return result;
  };

  std::vector<HGridCell3D> best = route;
  double best_cost = cost(best);
  bool changed = true;
  while (changed) {
    changed = false;
    for (std::size_t first = 0; first + 1 < best.size(); ++first) {
      for (std::size_t last = first + 2; last <= best.size(); ++last) {
        std::vector<HGridCell3D> candidate = best;
        std::reverse(candidate.begin() + static_cast<std::ptrdiff_t>(first),
                     candidate.begin() + static_cast<std::ptrdiff_t>(last));
        const double candidate_cost = cost(candidate);
        if (candidate_cost + 1.0e-9 < best_cost) {
          best = std::move(candidate);
          best_cost = candidate_cost;
          changed = true;
        }
      }
    }
  }
  return best;
}

}  // namespace racer_3d_cpp
