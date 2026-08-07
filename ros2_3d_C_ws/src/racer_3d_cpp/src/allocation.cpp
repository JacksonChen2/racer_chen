#include "racer_3d_cpp/allocation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

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

double routeCost(const std::vector<HGridCell3D> &route, const Point3 &start,
                 const HGridDistanceFunction &distance_function) {
  Point3 cursor = start;
  double result = 0.0;
  for (const auto &cell : route) {
    result += distance(cursor, cell.centroid, distance_function);
    cursor = cell.centroid;
  }
  return result;
}

std::vector<HGridCell3D> nearestRoute(
    const std::vector<HGridCell3D> &cells, const Point3 &start,
    const HGridDistanceFunction &distance_function) {
  std::vector<HGridCell3D> remaining = cells;
  std::vector<HGridCell3D> result;
  result.reserve(cells.size());
  Point3 cursor = start;
  while (!remaining.empty()) {
    std::size_t selected = 0;
    double best =
        distance(cursor, remaining.front().centroid, distance_function);
    for (std::size_t index = 1; index < remaining.size(); ++index) {
      const double candidate =
          distance(cursor, remaining[index].centroid, distance_function);
      if (candidate < best) {
        selected = index;
        best = candidate;
      }
    }
    result.push_back(remaining[selected]);
    cursor = remaining[selected].centroid;
    remaining.erase(remaining.begin() +
                    static_cast<std::ptrdiff_t>(selected));
  }
  return result;
}

struct ExactRoute {
  double cost{};
  std::vector<int> indices;
};

std::vector<ExactRoute> allExactOpenRoutes(
    const std::vector<HGridCell3D> &cells, const Point3 &start,
    const HGridDistanceFunction &distance_function) {
  const std::size_t count = cells.size();
  const std::size_t route_count = std::size_t{1} << count;
  std::vector<ExactRoute> routes(route_count);
  if (count == 0) {
    return routes;
  }

  std::vector<double> start_cost(count, 0.0);
  std::vector<std::vector<double>> pair_cost(
      count, std::vector<double>(count, 0.0));
  for (std::size_t first = 0; first < count; ++first) {
    start_cost[first] =
        distance(start, cells[first].centroid, distance_function);
    for (std::size_t second = first + 1; second < count; ++second) {
      const double value = distance(cells[first].centroid,
                                    cells[second].centroid,
                                    distance_function);
      pair_cost[first][second] = value;
      pair_cost[second][first] = value;
    }
  }

  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<std::vector<double>> dynamic(
      route_count, std::vector<double>(count, infinity));
  std::vector<std::vector<int>> parent(route_count,
                                       std::vector<int>(count, -1));
  for (std::size_t mask = 1; mask < route_count; ++mask) {
    for (std::size_t last = 0; last < count; ++last) {
      const std::size_t bit = std::size_t{1} << last;
      if ((mask & bit) == 0) {
        continue;
      }
      const std::size_t previous_mask = mask ^ bit;
      if (previous_mask == 0) {
        dynamic[mask][last] = start_cost[last];
        continue;
      }
      double best = infinity;
      int best_previous = -1;
      for (std::size_t candidate = 0; candidate < count; ++candidate) {
        if ((previous_mask & (std::size_t{1} << candidate)) == 0) {
          continue;
        }
        const double value =
            dynamic[previous_mask][candidate] + pair_cost[candidate][last];
        // Python's min((cost, index), ...) chooses the lower index on ties.
        if (value < best ||
            (value == best &&
             static_cast<int>(candidate) < best_previous)) {
          best = value;
          best_previous = static_cast<int>(candidate);
        }
      }
      dynamic[mask][last] = best;
      parent[mask][last] = best_previous;
    }

    double best_cost = infinity;
    int final = -1;
    for (std::size_t last = 0; last < count; ++last) {
      if ((mask & (std::size_t{1} << last)) == 0) {
        continue;
      }
      const double value = dynamic[mask][last];
      if (value < best_cost ||
          (value == best_cost && static_cast<int>(last) < final)) {
        best_cost = value;
        final = static_cast<int>(last);
      }
    }

    std::vector<int> indices;
    std::size_t cursor_mask = mask;
    int cursor = final;
    while (cursor >= 0) {
      indices.push_back(cursor);
      const int previous =
          parent[cursor_mask][static_cast<std::size_t>(cursor)];
      cursor_mask ^= std::size_t{1} << static_cast<std::size_t>(cursor);
      cursor = previous;
    }
    std::reverse(indices.begin(), indices.end());
    routes[mask] = ExactRoute{best_cost, std::move(indices)};
  }
  return routes;
}

std::vector<std::string> cellIds(
    const std::vector<HGridCell3D> &cells) {
  std::vector<std::string> ids;
  ids.reserve(cells.size());
  for (const auto &cell : cells) {
    ids.push_back(cell.id());
  }
  return ids;
}

std::vector<HGridCell3D> cellsForRoute(
    const std::vector<HGridCell3D> &cells,
    const std::vector<int> &indices) {
  std::vector<HGridCell3D> result;
  result.reserve(indices.size());
  for (const int index : indices) {
    result.push_back(cells[static_cast<std::size_t>(index)]);
  }
  return result;
}

}  // namespace

Partition3D capacityPartition(
    const std::vector<HGridCell3D> &cells, const Point3 &first_start,
    const Point3 &second_start, double imbalance,
    const HGridDistanceFunction &distance_function) {
  if (cells.empty()) {
    return {};
  }

  int total_demand = 0;
  for (const auto &cell : cells) {
    total_demand += cell.demand();
  }

  if (cells.size() <= 10) {
    const std::size_t route_count = std::size_t{1} << cells.size();
    const std::size_t full_mask = route_count - 1;
    const auto first_routes =
        allExactOpenRoutes(cells, first_start, distance_function);
    const auto second_routes =
        allExactOpenRoutes(cells, second_start, distance_function);

    bool have_best = false;
    Partition3D best;
    for (std::size_t mask = 0; mask < route_count; ++mask) {
      std::vector<HGridCell3D> first;
      std::vector<HGridCell3D> second;
      int first_demand = 0;
      for (std::size_t index = 0; index < cells.size(); ++index) {
        if ((mask & (std::size_t{1} << index)) != 0) {
          first.push_back(cells[index]);
          first_demand += cells[index].demand();
        } else {
          second.push_back(cells[index]);
        }
      }
      const int second_demand = total_demand - first_demand;
      const double allowed =
          std::max(1.0, imbalance * static_cast<double>(total_demand));
      const double balance_error =
          std::abs(static_cast<double>(first_demand - second_demand));
      const auto &first_route = first_routes[mask];
      const auto &second_route = second_routes[full_mask ^ mask];
      const double objective =
          first_route.cost + second_route.cost +
          0.02 * std::max(0.0, balance_error - allowed);

      if (!have_best || objective < best.cost) {
        best.first = cellIds(first);
        best.second = cellIds(second);
        best.firstRoute =
            cellIds(cellsForRoute(cells, first_route.indices));
        best.secondRoute =
            cellIds(cellsForRoute(cells, second_route.indices));
        best.firstDemand = first_demand;
        best.secondDemand = second_demand;
        best.cost = objective;
        have_best = true;
      }
    }
    return best;
  }

  // Deterministic bounded fallback for unusually large pairwise active sets.
  std::vector<HGridCell3D> ordered = cells;
  std::stable_sort(ordered.begin(), ordered.end(),
                   [](const HGridCell3D &first,
                      const HGridCell3D &second) {
                     return first.demand() > second.demand();
                   });
  std::vector<HGridCell3D> first;
  std::vector<HGridCell3D> second;
  int demand[2] = {0, 0};
  for (const auto &cell : ordered) {
    const double first_score =
        (cell.centroid - first_start).norm() + 0.01 * demand[0];
    const double second_score =
        (cell.centroid - second_start).norm() + 0.01 * demand[1];
    const int owner = second_score < first_score ? 1 : 0;
    (owner == 0 ? first : second).push_back(cell);
    demand[owner] += cell.demand();
  }
  const auto first_route =
      nearestRoute(first, first_start, distance_function);
  const auto second_route =
      nearestRoute(second, second_start, distance_function);
  Partition3D result;
  result.first = cellIds(first);
  result.second = cellIds(second);
  result.firstRoute = cellIds(first_route);
  result.secondRoute = cellIds(second_route);
  result.firstDemand = demand[0];
  result.secondDemand = demand[1];
  result.cost = routeCost(first_route, first_start, distance_function) +
                routeCost(second_route, second_start, distance_function);
  return result;
}

}  // namespace racer_3d_cpp
