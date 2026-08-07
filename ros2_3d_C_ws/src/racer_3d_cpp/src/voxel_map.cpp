#include "racer_3d_cpp/voxel_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace racer_3d_cpp {
namespace {

constexpr double kEdtInfinity = 1.0e30;

std::vector<double> edt1D(const std::vector<double> &input) {
  const int count = static_cast<int>(input.size());
  std::vector<double> output(
      input.size(), std::numeric_limits<double>::infinity());
  if (count == 0) {
    return output;
  }

  std::vector<int> sites;
  sites.reserve(input.size());
  for (int index = 0; index < count; ++index) {
    if (std::isfinite(input[static_cast<std::size_t>(index)]) &&
        input[static_cast<std::size_t>(index)] < kEdtInfinity) {
      sites.push_back(index);
    }
  }
  if (sites.empty()) {
    return output;
  }

  std::vector<int> envelope(sites.size());
  std::vector<double> boundaries(
      sites.size() + 1U, std::numeric_limits<double>::infinity());
  int top = 0;
  envelope[0] = sites[0];
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();

  for (std::size_t site_index = 1; site_index < sites.size(); ++site_index) {
    const int q = sites[site_index];
    double intersection = 0.0;
    while (true) {
      const int previous = envelope[static_cast<std::size_t>(top)];
      intersection =
          ((input[static_cast<std::size_t>(q)] +
            static_cast<double>(q) * q) -
           (input[static_cast<std::size_t>(previous)] +
            static_cast<double>(previous) * previous)) /
          (2.0 * static_cast<double>(q - previous));
      if (intersection > boundaries[static_cast<std::size_t>(top)] ||
          top == 0) {
        break;
      }
      --top;
    }
    ++top;
    envelope[static_cast<std::size_t>(top)] = q;
    boundaries[static_cast<std::size_t>(top)] = intersection;
    boundaries[static_cast<std::size_t>(top + 1)] =
        std::numeric_limits<double>::infinity();
  }

  top = 0;
  for (int q = 0; q < count; ++q) {
    while (boundaries[static_cast<std::size_t>(top + 1)] <
           static_cast<double>(q)) {
      ++top;
    }
    const int site = envelope[static_cast<std::size_t>(top)];
    const double difference = static_cast<double>(q - site);
    output[static_cast<std::size_t>(q)] =
        difference * difference + input[static_cast<std::size_t>(site)];
  }
  return output;
}

}  // namespace

VoxelMap::VoxelMap(
    double resolution, const Point3 &origin, const Point3 &size)
    : resolution_(resolution), origin_(origin), size_(size) {
  if (!std::isfinite(resolution_) || resolution_ <= 0.0) {
    throw std::invalid_argument("voxel resolution must be positive and finite");
  }
  if (!(origin_.array().isFinite().all()) ||
      !(size_.array().isFinite().all()) ||
      (size_.array() <= 0.0).any()) {
    throw std::invalid_argument("voxel origin/size is invalid");
  }
  nx_ = static_cast<int>(std::ceil(size_.x() / resolution_));
  ny_ = static_cast<int>(std::ceil(size_.y() / resolution_));
  nz_ = static_cast<int>(std::ceil(size_.z() / resolution_));
  const std::size_t count =
      static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) *
      static_cast<std::size_t>(nz_);
  log_odds_.assign(count, 0);
  observations_.assign(count, 0U);
  states_cache_.assign(count, UNKNOWN);
}

bool VoxelMap::inBounds(const GridIndex3 &cell) const noexcept {
  return cell.x >= 0 && cell.x < nx_ && cell.y >= 0 && cell.y < ny_ &&
         cell.z >= 0 && cell.z < nz_;
}

std::size_t VoxelMap::flatIndex(const GridIndex3 &cell) const noexcept {
  return (static_cast<std::size_t>(cell.z) * static_cast<std::size_t>(ny_) +
          static_cast<std::size_t>(cell.y)) *
             static_cast<std::size_t>(nx_) +
         static_cast<std::size_t>(cell.x);
}

std::optional<GridIndex3> VoxelMap::worldToGrid(
    const Point3 &point) const {
  if (!(point.array().isFinite().all())) {
    return std::nullopt;
  }
  GridIndex3 cell{
      static_cast<int>(std::floor((point.x() - origin_.x()) / resolution_)),
      static_cast<int>(std::floor((point.y() - origin_.y()) / resolution_)),
      static_cast<int>(std::floor((point.z() - origin_.z()) / resolution_))};
  if (!inBounds(cell)) {
    return std::nullopt;
  }
  return cell;
}

Point3 VoxelMap::gridToWorld(const GridIndex3 &cell) const {
  return origin_ +
         resolution_ *
             Point3(
                 static_cast<double>(cell.x) + 0.5,
                 static_cast<double>(cell.y) + 0.5,
                 static_cast<double>(cell.z) + 0.5);
}

std::vector<GridIndex3> VoxelMap::rayCells(
    const Point3 &start, const Point3 &end) const {
  const double distance = (end - start).norm();
  const int samples = std::max(
      1, static_cast<int>(std::ceil(distance / (0.35 * resolution_))));
  std::vector<GridIndex3> result;
  result.reserve(static_cast<std::size_t>(samples + 1));
  std::optional<GridIndex3> previous;
  for (int index = 0; index <= samples; ++index) {
    const double ratio =
        static_cast<double>(index) / static_cast<double>(samples);
    const auto cell = worldToGrid(start + ratio * (end - start));
    if (cell && (!previous || *cell != *previous)) {
      result.push_back(*cell);
      previous = cell;
    }
  }
  return result;
}

void VoxelMap::observeBulk(std::vector<std::size_t> cells, int delta) {
  if (cells.empty()) {
    return;
  }
  std::sort(cells.begin(), cells.end());
  std::size_t cursor = 0U;
  while (cursor < cells.size()) {
    const std::size_t index = cells[cursor];
    std::size_t next = cursor + 1U;
    while (next < cells.size() && cells[next] == index) {
      ++next;
    }
    if (index < log_odds_.size()) {
      const std::size_t occurrences = next - cursor;
      const long long updated =
          static_cast<long long>(log_odds_[index]) +
          static_cast<long long>(delta) *
              static_cast<long long>(occurrences);
      log_odds_[index] = static_cast<std::int16_t>(
          std::clamp<long long>(updated, -30LL, 30LL));
      const std::uint64_t observed =
          static_cast<std::uint64_t>(observations_[index]) + occurrences;
      observations_[index] = static_cast<std::uint16_t>(
          std::min<std::uint64_t>(
              observed, std::numeric_limits<std::uint16_t>::max()));
    }
    cursor = next;
  }
  invalidateCaches();
}

void VoxelMap::updatePointCloud(
    const Point3 &sensor_origin, const std::vector<Point3> &points_world,
    double maximum_range, const std::vector<std::uint8_t> &hit_mask,
    std::size_t maximum_rays) {
  if (!hit_mask.empty() && hit_mask.size() != points_world.size()) {
    throw std::invalid_argument(
        "point-cloud hit mask must be empty or match point count");
  }
  if (!std::isfinite(maximum_range) || maximum_range <= 0.0 ||
      maximum_rays == 0U) {
    return;
  }

  std::vector<std::size_t> selected;
  const std::size_t selected_count =
      std::min(points_world.size(), maximum_rays);
  selected.reserve(selected_count);
  if (points_world.size() <= maximum_rays) {
    for (std::size_t index = 0; index < points_world.size(); ++index) {
      selected.push_back(index);
    }
  } else if (selected_count == 1U) {
    selected.push_back(0U);
  } else {
    for (std::size_t index = 0; index < selected_count; ++index) {
      // NumPy linspace(..., dtype=int64) truncates each positive sample.
      const double value =
          static_cast<double>(index) *
          static_cast<double>(points_world.size() - 1U) /
          static_cast<double>(selected_count - 1U);
      selected.push_back(static_cast<std::size_t>(value));
    }
  }

  std::vector<std::size_t> free_evidence;
  std::vector<std::size_t> occupied_evidence;
  free_evidence.reserve(selected_count * 24U);
  occupied_evidence.reserve(selected_count);
  if (const auto start = worldToGrid(sensor_origin)) {
    const auto flat = flatIndex(*start);
    free_evidence.insert(free_evidence.end(), 3U, flat);
  }

  for (const std::size_t input_index : selected) {
    const Point3 &point = points_world[input_index];
    if (!(point.array().isFinite().all())) {
      continue;
    }
    const Point3 vector = point - sensor_origin;
    const double distance = vector.norm();
    if (!std::isfinite(distance) || distance < 0.03) {
      continue;
    }
    const double clipped = std::min(distance, maximum_range);
    const Point3 endpoint = sensor_origin + (clipped / distance) * vector;
    const auto cells = rayCells(sensor_origin, endpoint);
    if (cells.empty()) {
      continue;
    }
    const bool requested_hit =
        hit_mask.empty() || hit_mask[input_index] != 0U;
    const bool actual_hit =
        requested_hit && distance < maximum_range - 0.03;
    const std::size_t free_count =
        actual_hit ? cells.size() - 1U : cells.size();
    for (std::size_t index = 0; index < free_count; ++index) {
      free_evidence.push_back(flatIndex(cells[index]));
    }
    if (actual_hit) {
      const Point3 occupied_point =
          sensor_origin +
          (std::min(
               maximum_range, distance + 0.5 * resolution_) /
           distance) *
              vector;
      auto occupied = worldToGrid(occupied_point);
      if (!occupied) {
        occupied = cells.back();
      }
      occupied_evidence.push_back(flatIndex(*occupied));
    }
  }
  observeBulk(std::move(free_evidence), -2);
  observeBulk(std::move(occupied_evidence), 8);
}

void VoxelMap::updatePointCloud(
    const Point3 &sensor_origin, const std::vector<Point3> &points_world,
    double maximum_range, const std::vector<bool> &hit_mask,
    std::size_t maximum_rays) {
  std::vector<std::uint8_t> values;
  values.reserve(hit_mask.size());
  for (const bool hit : hit_mask) {
    values.push_back(hit ? 1U : 0U);
  }
  updatePointCloud(
      sensor_origin, points_world, maximum_range, values, maximum_rays);
}

const std::vector<std::int8_t> &VoxelMap::states() const {
  if (states_valid_) {
    return states_cache_;
  }
  if (states_cache_.size() != log_odds_.size()) {
    states_cache_.resize(log_odds_.size());
  }
  for (std::size_t index = 0; index < log_odds_.size(); ++index) {
    std::int8_t value = UNKNOWN;
    if (observations_[index] > 0U) {
      if (log_odds_[index] <= -2) {
        value = FREE;
      } else if (log_odds_[index] >= 2) {
        value = OCCUPIED;
      }
    }
    states_cache_[index] = value;
  }
  states_valid_ = true;
  return states_cache_;
}

std::int8_t VoxelMap::stateAt(const GridIndex3 &cell) const {
  if (!inBounds(cell)) {
    return UNKNOWN;
  }
  return states()[flatIndex(cell)];
}

void VoxelMap::setStates(const std::vector<std::int8_t> &values) {
  if (values.size() != voxelCount()) {
    throw std::invalid_argument("voxel state array shape/size mismatch");
  }
  std::fill(observations_.begin(), observations_.end(), 0U);
  std::fill(log_odds_.begin(), log_odds_.end(), 0);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (values[index] >= 0) {
      observations_[index] = 1U;
    }
    if (values[index] == FREE) {
      log_odds_[index] = -10;
    } else if (values[index] == OCCUPIED) {
      log_odds_[index] = 10;
    }
  }
  invalidateCaches();
}

void VoxelMap::merge(const std::vector<std::int8_t> &values) {
  if (values.size() != voxelCount()) {
    return;
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (values[index] < 0) {
      continue;
    }
    observations_[index] = std::max<std::uint16_t>(
        observations_[index], 1U);
    if (values[index] == FREE && log_odds_[index] < 2) {
      log_odds_[index] = std::min<std::int16_t>(log_odds_[index], -8);
    } else if (values[index] == OCCUPIED) {
      log_odds_[index] = std::max<std::int16_t>(log_odds_[index], 10);
    }
  }
  invalidateCaches();
}

double VoxelMap::coverage() const {
  const auto &values = states();
  const std::size_t known = static_cast<std::size_t>(std::count_if(
      values.begin(), values.end(),
      [](std::int8_t value) { return value != UNKNOWN; }));
  return static_cast<double>(known) /
         static_cast<double>(std::max<std::size_t>(1U, values.size()));
}

std::vector<float> VoxelMap::squaredDistanceTransform(
    const std::vector<std::uint8_t> &source) const {
  std::vector<double> first(voxelCount(), kEdtInfinity);
  for (std::size_t index = 0; index < source.size(); ++index) {
    if (source[index] != 0U) {
      first[index] = 0.0;
    }
  }
  std::vector<double> second(voxelCount(), kEdtInfinity);
  std::vector<double> third(voxelCount(), kEdtInfinity);
  std::vector<double> line;

  line.resize(static_cast<std::size_t>(nx_));
  for (int z = 0; z < nz_; ++z) {
    for (int y = 0; y < ny_; ++y) {
      for (int x = 0; x < nx_; ++x) {
        line[static_cast<std::size_t>(x)] =
            first[flatIndex({x, y, z})];
      }
      const auto transformed = edt1D(line);
      for (int x = 0; x < nx_; ++x) {
        second[flatIndex({x, y, z})] =
            transformed[static_cast<std::size_t>(x)];
      }
    }
  }

  line.resize(static_cast<std::size_t>(ny_));
  for (int z = 0; z < nz_; ++z) {
    for (int x = 0; x < nx_; ++x) {
      for (int y = 0; y < ny_; ++y) {
        line[static_cast<std::size_t>(y)] =
            second[flatIndex({x, y, z})];
      }
      const auto transformed = edt1D(line);
      for (int y = 0; y < ny_; ++y) {
        third[flatIndex({x, y, z})] =
            transformed[static_cast<std::size_t>(y)];
      }
    }
  }

  line.resize(static_cast<std::size_t>(nz_));
  std::vector<float> result(voxelCount());
  for (int y = 0; y < ny_; ++y) {
    for (int x = 0; x < nx_; ++x) {
      for (int z = 0; z < nz_; ++z) {
        line[static_cast<std::size_t>(z)] =
            third[flatIndex({x, y, z})];
      }
      const auto transformed = edt1D(line);
      for (int z = 0; z < nz_; ++z) {
        result[flatIndex({x, y, z})] =
            std::isfinite(transformed[static_cast<std::size_t>(z)])
                ? static_cast<float>(
                      transformed[static_cast<std::size_t>(z)])
                : std::numeric_limits<float>::infinity();
      }
    }
  }
  return result;
}

const std::vector<float> &VoxelMap::esdf(bool unknown_is_occupied) const {
  bool &valid =
      unknown_is_occupied ? esdf_unknown_valid_ : esdf_known_valid_;
  std::vector<float> &cache =
      unknown_is_occupied ? esdf_unknown_cache_ : esdf_known_cache_;
  if (valid) {
    return cache;
  }

  const auto &values = states();
  std::vector<std::uint8_t> source(voxelCount(), 0U);
  std::vector<std::uint8_t> complement(voxelCount(), 0U);
  for (std::size_t index = 0; index < values.size(); ++index) {
    const bool selected =
        values[index] == OCCUPIED ||
        (unknown_is_occupied && values[index] == UNKNOWN);
    source[index] = selected ? 1U : 0U;
    complement[index] = selected ? 0U : 1U;
  }
  const auto outside_squared = squaredDistanceTransform(source);
  const auto inside_squared = squaredDistanceTransform(complement);
  cache.resize(voxelCount());
  for (std::size_t index = 0; index < cache.size(); ++index) {
    const float squared =
        source[index] != 0U ? inside_squared[index] : outside_squared[index];
    const float distance =
        std::isfinite(squared)
            ? static_cast<float>(
                  std::sqrt(std::max(0.0F, squared)) * resolution_)
            : std::numeric_limits<float>::infinity();
    cache[index] = source[index] != 0U ? -distance : distance;
  }
  valid = true;
  return cache;
}

double VoxelMap::distanceAt(
    const Point3 &point, bool unknown_is_occupied) const {
  const Point3 coordinate =
      (point - origin_) / resolution_ - Point3::Constant(0.5);
  const Point3 maximum(
      static_cast<double>(nx_ - 1),
      static_cast<double>(ny_ - 1),
      static_cast<double>(nz_ - 1));
  if (!(coordinate.array().isFinite().all()) ||
      (coordinate.array() < 0.0).any() ||
      (coordinate.array() > maximum.array()).any()) {
    return -std::numeric_limits<double>::infinity();
  }

  const GridIndex3 lower{
      static_cast<int>(std::floor(coordinate.x())),
      static_cast<int>(std::floor(coordinate.y())),
      static_cast<int>(std::floor(coordinate.z()))};
  const GridIndex3 upper{
      std::min(nx_ - 1, lower.x + 1),
      std::min(ny_ - 1, lower.y + 1),
      std::min(nz_ - 1, lower.z + 1)};
  const Point3 ratio(
      coordinate.x() - lower.x,
      coordinate.y() - lower.y,
      coordinate.z() - lower.z);
  const auto &field = esdf(unknown_is_occupied);
  double result = 0.0;
  for (int dz = 0; dz <= 1; ++dz) {
    const int z = dz == 0 ? lower.z : upper.z;
    const double wz = dz == 0 ? 1.0 - ratio.z() : ratio.z();
    for (int dy = 0; dy <= 1; ++dy) {
      const int y = dy == 0 ? lower.y : upper.y;
      const double wy = dy == 0 ? 1.0 - ratio.y() : ratio.y();
      for (int dx = 0; dx <= 1; ++dx) {
        const int x = dx == 0 ? lower.x : upper.x;
        const double wx = dx == 0 ? 1.0 - ratio.x() : ratio.x();
        const double weight = wx * wy * wz;
        if (weight <= 0.0) {
          continue;
        }
        const float value = field[flatIndex({x, y, z})];
        if (std::isinf(value)) {
          return value;
        }
        result += static_cast<double>(value) * weight;
      }
    }
  }
  return result;
}

Point3 VoxelMap::esdfGradient(
    const Point3 &point, bool unknown_is_occupied) const {
  if (!worldToGrid(point)) {
    return Point3::Zero();
  }
  Point3 gradient = Point3::Zero();
  const double step = 0.35 * resolution_;
  for (int axis = 0; axis < 3; ++axis) {
    Point3 lower = point;
    Point3 upper = point;
    lower[axis] -= step;
    upper[axis] += step;
    const double low = distanceAt(lower, unknown_is_occupied);
    const double high = distanceAt(upper, unknown_is_occupied);
    if (std::isfinite(low) && std::isfinite(high)) {
      gradient[axis] = (high - low) / (2.0 * step);
    }
  }
  return gradient;
}

std::vector<std::uint8_t> VoxelMap::inflatedBlocked(
    double clearance, bool unknown_is_blocked) const {
  const auto &field = esdf(unknown_is_blocked);
  std::vector<std::uint8_t> blocked(field.size(), 0U);
  for (std::size_t index = 0; index < field.size(); ++index) {
    blocked[index] = field[index] < clearance ? 1U : 0U;
  }
  return blocked;
}

std::vector<std::uint8_t> VoxelMap::frontierMask() const {
  const auto &values = states();
  std::vector<std::uint8_t> frontier(voxelCount(), 0U);
  static constexpr std::array<GridIndex3, 6> neighbors{{
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
      {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
  }};
  for (int z = 0; z < nz_; ++z) {
    for (int y = 0; y < ny_; ++y) {
      for (int x = 0; x < nx_; ++x) {
        const GridIndex3 cell{x, y, z};
        if (values[flatIndex(cell)] != FREE) {
          continue;
        }
        for (const auto &delta : neighbors) {
          const GridIndex3 adjacent{
              x + delta.x, y + delta.y, z + delta.z};
          if (inBounds(adjacent) &&
              values[flatIndex(adjacent)] == UNKNOWN) {
            frontier[flatIndex(cell)] = 1U;
            break;
          }
        }
      }
    }
  }
  return frontier;
}

std::vector<std::vector<GridIndex3>> VoxelMap::frontierClusters(
    std::size_t minimum_size) const {
  const auto frontier = frontierMask();
  std::vector<std::uint8_t> visited(voxelCount(), 0U);
  std::vector<std::vector<GridIndex3>> clusters;
  for (int z = 0; z < nz_; ++z) {
    for (int y = 0; y < ny_; ++y) {
      for (int x = 0; x < nx_; ++x) {
        const GridIndex3 seed{x, y, z};
        const std::size_t seed_index = flatIndex(seed);
        if (frontier[seed_index] == 0U || visited[seed_index] != 0U) {
          continue;
        }
        std::deque<GridIndex3> queue;
        queue.push_back(seed);
        visited[seed_index] = 1U;
        std::vector<GridIndex3> cluster;
        while (!queue.empty()) {
          const GridIndex3 current = queue.front();
          queue.pop_front();
          cluster.push_back(current);
          for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) {
                  continue;
                }
                const GridIndex3 adjacent{
                    current.x + dx, current.y + dy, current.z + dz};
                if (!inBounds(adjacent)) {
                  continue;
                }
                const std::size_t index = flatIndex(adjacent);
                if (frontier[index] != 0U && visited[index] == 0U) {
                  visited[index] = 1U;
                  queue.push_back(adjacent);
                }
              }
            }
          }
        }
        if (cluster.size() >= minimum_size) {
          clusters.push_back(std::move(cluster));
        }
      }
    }
  }
  return clusters;
}

int VoxelMap::informationGain(const Point3 &point, double radius_m) const {
  const auto center = worldToGrid(point);
  if (!center) {
    return 0;
  }
  const int radius =
      static_cast<int>(std::ceil(radius_m / resolution_));
  const auto &values = states();
  int gain = 0;
  for (int z = std::max(0, center->z - radius);
       z < std::min(nz_, center->z + radius + 1); ++z) {
    for (int y = std::max(0, center->y - radius);
         y < std::min(ny_, center->y + radius + 1); ++y) {
      for (int x = std::max(0, center->x - radius);
           x < std::min(nx_, center->x + radius + 1); ++x) {
        const int dx = x - center->x;
        const int dy = y - center->y;
        const int dz = z - center->z;
        if (dx * dx + dy * dy + dz * dz <= radius * radius &&
            values[flatIndex({x, y, z})] == UNKNOWN) {
          ++gain;
        }
      }
    }
  }
  return gain;
}

int VoxelMap::visibleUnknownGain(
    const Point3 &viewpoint, const std::vector<GridIndex3> &cluster,
    std::size_t maximum_rays) const {
  if (cluster.empty()) {
    return 0;
  }
  const auto &values = states();
  const std::size_t divisor = std::max<std::size_t>(1U, maximum_rays);
  const std::size_t stride =
      std::max<std::size_t>(1U, cluster.size() / divisor);
  std::unordered_set<GridIndex3, GridIndex3Hash> gain_cells;
  for (std::size_t index = 0; index < cluster.size(); index += stride) {
    const Point3 target = gridToWorld(cluster[index]);
    const Point3 ray = target - viewpoint;
    const double distance = ray.norm();
    if (distance < 1.0e-6) {
      continue;
    }
    const Point3 endpoint = target + (2.0 / distance) * ray;
    const auto crossed = rayCells(viewpoint, endpoint);
    for (const auto &candidate : crossed) {
      const std::int8_t value = values[flatIndex(candidate)];
      if (value == OCCUPIED) {
        break;
      }
      if (value == UNKNOWN) {
        gain_cells.insert(candidate);
      }
    }
  }
  return static_cast<int>(gain_cells.size());
}

void VoxelMap::invalidateCaches() noexcept {
  states_valid_ = false;
  esdf_unknown_valid_ = false;
  esdf_known_valid_ = false;
}

}  // namespace racer_3d_cpp
