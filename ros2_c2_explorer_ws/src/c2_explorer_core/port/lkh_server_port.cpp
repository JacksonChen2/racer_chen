#include <lkh_mtsp_solver/lkh3_interface.h>
#include <c2_explorer_msgs/srv/solve_mtsp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <array>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <sys/wait.h>
#include <vector>

namespace {

bool forcePrecisionOne(const std::string &parameter_file,
                       const rclcpp::Logger &logger) {
  std::ifstream input(parameter_file);
  if (!input.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open par file for precision patch: %s",
                 parameter_file.c_str());
    return false;
  }

  std::vector<std::string> lines;
  lines.reserve(64);
  std::string line;
  while (std::getline(input, line)) {
    if (line.rfind("PRECISION", 0) == 0) continue;
    lines.push_back(line);
  }
  input.close();

  std::ofstream output(parameter_file, std::ios::trunc);
  if (!output.is_open()) {
    RCLCPP_ERROR(logger, "Failed to rewrite par file for precision patch: %s",
                 parameter_file.c_str());
    return false;
  }
  for (const auto &entry : lines) output << entry << '\n';
  output << "PRECISION = 1\n";
  output.close();
  return !output.fail();
}

}  // namespace

class LkhServer final : public rclcpp::Node {
 public:
  LkhServer() : Node("c2_explorer_lkh_server") {
    const int drone_id = declare_parameter<int>("exploration.drone_id", 1);
    const int problem_id = declare_parameter<int>("exploration.problem_id", 1);
    const std::string directory =
        declare_parameter<std::string>("exploration.mtsp_dir", "/tmp/c2_explorer_lkh");
    lkh_executable_ = declare_parameter<std::string>(
        "exploration.lkh_executable", "LKH");
    parameter_files_[1] = directory + "/amtsp_" + std::to_string(drone_id) + ".par";
    parameter_files_[2] = directory + "/amtsp2_" + std::to_string(drone_id) + ".par";
    parameter_files_[3] = directory + "/amtsp3_" + std::to_string(drone_id) + ".par";
    if (problem_id == 1) {
      service_name_ = "/solve_tsp_" + std::to_string(drone_id);
    } else {
      service_name_ = "/solve_acvrp_" + std::to_string(drone_id);
    }
    service_ = create_service<c2_explorer_msgs::srv::SolveMTSP>(
        service_name_,
        [this](const std::shared_ptr<c2_explorer_msgs::srv::SolveMTSP::Request> request,
               std::shared_ptr<c2_explorer_msgs::srv::SolveMTSP::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (request->prob < 1 || request->prob > 3) {
            response->empty = 0;
            return;
          }
          int result = EXIT_FAILURE;
          if (request->prob == 3) {
            // This is deliberately a standalone process, matching the
            // original mtsp_node's `/usr/local/bin/LKH <amtsp3.par>` path.
            // LKH uses process-global state and is not re-entrant across
            // successive ACVRP problems in one long-lived process.
            if (forcePrecisionOne(parameter_files_[3], get_logger())) {
              const std::string command =
                  lkh_executable_ + " \"" + parameter_files_[3] + "\"";
              const int status = std::system(command.c_str());
              result = status != -1 && WIFEXITED(status)
                           ? WEXITSTATUS(status)
                           : EXIT_FAILURE;
              if (result != EXIT_SUCCESS) {
                RCLCPP_ERROR(get_logger(),
                             "LKH process failed with status %d (par: %s)",
                             status, parameter_files_[3].c_str());
              }
            }
          } else {
            solveMTSPWithLKH3(parameter_files_[request->prob].c_str());
            result = EXIT_SUCCESS;
          }
          response->empty =
              static_cast<std::uint8_t>(result == EXIT_SUCCESS ? 1 : 0);
        });
    RCLCPP_INFO(get_logger(), "C2-Explorer LKH-3 server ready: %s", service_name_.c_str());
  }

 private:
  std::array<std::string, 4> parameter_files_;
  std::string service_name_;
  std::string lkh_executable_;
  std::mutex mutex_;
  rclcpp::Service<c2_explorer_msgs::srv::SolveMTSP>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LkhServer>());
  rclcpp::shutdown();
  return 0;
}
