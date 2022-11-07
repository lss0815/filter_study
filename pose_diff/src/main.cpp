#include "pose_diff/include/state.h"

#include <ctime>
#include <iostream>
#include <random>

#include <Eigen/Core>

int main() {
  Eigen::Matrix4d pose_matrix;
  pose_matrix << -1, 0, 0, -0.1, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;
  pose_diff::State state(pose_matrix);

  Eigen::VectorXd state_6d = state.Get6DState();

  auto print_vector = [](const Eigen::VectorXd& vector) {
    for (int i = 0; i < vector.size(); i++) {
      std::cout << vector(i) << " ";
    }
    std::cout << "\n";
  };

  print_vector(state_6d);

  const double KRotStdDev = 0.05;
  const double kTransStdDev = 0.1;
  std::default_random_engine random_engine(time(nullptr));
  std::normal_distribution<double> rot_gaussian(0, KRotStdDev);
  std::normal_distribution<double> trans_gaussian(0, kTransStdDev);

  Eigen::VectorXd state_6d_with_noise(6);

  state_6d_with_noise(0) = state_6d(0) + rot_gaussian(random_engine);
  state_6d_with_noise(1) = state_6d(1) + rot_gaussian(random_engine);
  state_6d_with_noise(2) = state_6d(2) + rot_gaussian(random_engine);

  state_6d_with_noise(3) = state_6d(3) + trans_gaussian(random_engine);
  state_6d_with_noise(4) = state_6d(4) + trans_gaussian(random_engine);
  state_6d_with_noise(5) = state_6d(5) + trans_gaussian(random_engine);

  print_vector(state_6d_with_noise);

  return 0;
}
