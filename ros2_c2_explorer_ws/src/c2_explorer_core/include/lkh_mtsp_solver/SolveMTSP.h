#pragma once
#include <c2_explorer_msgs/srv/solve_mtsp.hpp>
namespace lkh_mtsp_solver {
struct SolveMTSP {
  using RosService = c2_explorer_msgs::srv::SolveMTSP;
  RosService::Request request;
  RosService::Response response;
};
}
