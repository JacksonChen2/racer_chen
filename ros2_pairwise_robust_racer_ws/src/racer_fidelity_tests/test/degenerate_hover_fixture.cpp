#include <poly_traj/polynomial_traj.h>

#include <cmath>
#include <iostream>

int main() {
  const Eigen::Vector3d position(2.06719, -11.1637, 3.5459);
  Eigen::MatrixXd positions(3, 3);
  positions.row(0) = position;
  positions.row(1) = position;
  positions.row(2) = position;

  // The repaired planner reuses the original endpoint-acceleration duration
  // and divides it over two polynomial segments.
  Eigen::VectorXd times(2);
  times << 0.25, 0.25;
  fast_planner::PolynomialTraj trajectory;
  fast_planner::PolynomialTraj::waypointsTraj(
      positions, Eigen::Vector3d(0.08, -0.04, 0.02),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), times, trajectory);

  const double duration = trajectory.getTotalTime();
  if (!std::isfinite(duration) || std::abs(duration - 0.5) > 1.0e-12) return 1;
  for (double time = 0.0; time <= duration; time += 0.01) {
    for (int derivative = 0; derivative <= 2; ++derivative) {
      if (!trajectory.evaluate(time, derivative).allFinite()) return 2;
    }
  }
  if ((trajectory.evaluate(0.0, 0) - position).norm() > 1.0e-9) return 3;
  if ((trajectory.evaluate(duration, 0) - position).norm() > 1.0e-8) return 4;

  std::cout << "PASS: finite two-segment hover trajectory, duration="
            << duration << std::endl;
  return 0;
}
