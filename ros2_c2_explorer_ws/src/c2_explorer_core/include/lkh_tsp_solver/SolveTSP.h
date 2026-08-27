#pragma once
#include <c2_explorer_msgs/srv/solve_tsp.hpp>
namespace lkh_tsp_solver {
struct SolveTSP {
  using RosService = c2_explorer_msgs::srv::SolveTSP;
  RosService::Request request;
  RosService::Response response;
};
}
