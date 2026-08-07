#pragma once

#include "racer_3d_cpp/hgrid.hpp"

#include <string>
#include <vector>

namespace racer_3d_cpp {

struct Partition3D {
  std::vector<std::string> first;
  std::vector<std::string> second;
  std::vector<std::string> firstRoute;
  std::vector<std::string> secondRoute;
  int firstDemand{};
  int secondDemand{};
  double cost{};
};

Partition3D capacityPartition(
    const std::vector<HGridCell3D> &cells, const Point3 &first_start,
    const Point3 &second_start, double imbalance = 0.20,
    const HGridDistanceFunction &distance_function = {});

}  // namespace racer_3d_cpp
