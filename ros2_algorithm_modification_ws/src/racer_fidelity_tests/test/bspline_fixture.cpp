#include <bspline/non_uniform_bspline.h>

#include <Eigen/Core>
#include <iomanip>
#include <iostream>
#include <vector>

using fast_planner::NonUniformBspline;

static void printVector(const Eigen::VectorXd &value) {
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << value(i);
  }
  std::cout << '\n';
}

static void printMatrix(const Eigen::MatrixXd &value) {
  std::cout << value.rows() << 'x' << value.cols() << '\n';
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    printVector(value.row(row));
  }
}

int main() {
  std::cout << std::scientific << std::setprecision(17);

  std::vector<Eigen::Vector3d> points{
      {-2.0, 1.0, 0.5}, {-1.1, 1.7, 0.9}, {0.2, 1.4, 1.4},
      {1.6, 0.1, 1.8}, {2.4, -1.2, 1.2}, {3.1, -1.8, 0.7}};
  std::vector<Eigen::Vector3d> boundary{
      {0.15, -0.05, 0.10}, {0.0, 0.0, 0.0},
      {0.20, 0.10, -0.08}, {0.0, 0.0, 0.0}};

  Eigen::MatrixXd control_points;
  NonUniformBspline::parameterizeToBspline(
      0.37, points, boundary, 3, control_points);
  printMatrix(control_points);

  NonUniformBspline spline(control_points, 3, 0.37);
  const double duration = spline.getTimeSum();
  std::cout << "duration," << duration << '\n';
  for (int i = 0; i <= 16; ++i) {
    std::cout << "position," << i << ',';
    printVector(spline.evaluateDeBoorT(duration * i / 16.0));
  }

  NonUniformBspline velocity = spline.getDerivative();
  NonUniformBspline acceleration = velocity.getDerivative();
  for (int i = 0; i <= 8; ++i) {
    const double t = duration * i / 8.0;
    std::cout << "velocity," << i << ',';
    printVector(velocity.evaluateDeBoorT(t));
    std::cout << "acceleration," << i << ',';
    printVector(acceleration.evaluateDeBoorT(t));
  }
  std::cout << "length," << spline.getLength(0.013) << '\n';
  std::cout << "jerk," << spline.getJerk() << '\n';
  return 0;
}
