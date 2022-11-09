#include "pose_diff/include/state.h"
#include "pose_diff/include/transform.h"

#include <ctime>
#include <iostream>
#include <random>

#include <Eigen/Core>

int main() {
  Eigen::Matrix4d pose_matrix;
  // T_c1_wrt_c2
  pose_matrix << -1, 0, 0, -0.1, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;
  pose_diff::State state(pose_matrix);

  Eigen::VectorXd state_6d = state.Get6DState();

  auto print_vector = [](const Eigen::VectorXd& vector) {
    for (int i = 0; i < vector.size(); i++) {
      std::cout << vector(i) << " ";
    }
    std::cout << "\n";
  };

  std::cout << "state of C1 is identity\n\n\n";

  std::cout << "state of C2\n";
  print_vector(state_6d);
  std::cout << "\n\n";

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

  std::cout << "state of C2 with noise\n";
  print_vector(state_6d_with_noise);
  std::cout << "\n\n";

  Eigen::Vector3d rot_vector = state_6d.head(3);
  Eigen::Matrix3d conversion_result =
      pose_diff::Rotation::AxisAngleToMatrix(rot_vector) - pose_matrix.block<3, 3>(0, 0);
  std::cout << "Matrix normalize R - R` (R` from axis-angle)\n";
  std::cout << conversion_result.norm() << "\n\n\n";

  const double kPointPosStdDev = 5.0;
  std::normal_distribution<double> point_pos_gaussian(0, kPointPosStdDev);
  std::vector<Eigen::Vector3d> c1_point_list(3);
  c1_point_list[0] << point_pos_gaussian(random_engine), point_pos_gaussian(random_engine),
      point_pos_gaussian(random_engine);
  c1_point_list[1] << point_pos_gaussian(random_engine), point_pos_gaussian(random_engine),
      point_pos_gaussian(random_engine);
  c1_point_list[2] << point_pos_gaussian(random_engine), point_pos_gaussian(random_engine),
      point_pos_gaussian(random_engine);

  std::cout << "Points at C1\n";
  print_vector(c1_point_list[0]);
  print_vector(c1_point_list[1]);
  print_vector(c1_point_list[2]);

  pose_diff::State state_diff_c2_wrt_c1 = state.GetInverse();
  Eigen::Matrix4d transformation_matrix_c2_wrt_c1 = state_diff_c2_wrt_c1.GetTransformationMatrix();
  std::vector<Eigen::Vector3d> c2_point_list(3);
  c2_point_list[0] = pose_diff::Transform::Transform3DPoint(transformation_matrix_c2_wrt_c1, c1_point_list[0]);
  c2_point_list[1] = pose_diff::Transform::Transform3DPoint(transformation_matrix_c2_wrt_c1, c1_point_list[1]);
  c2_point_list[2] = pose_diff::Transform::Transform3DPoint(transformation_matrix_c2_wrt_c1, c1_point_list[2]);
  std::cout << "\n\n";

  std::cout << "Points at C2\n";
  print_vector(c2_point_list[0]);
  print_vector(c2_point_list[1]);
  print_vector(c2_point_list[2]);
  std::cout << "\n\n";

  Eigen::Vector3d error_from_noise;
  pose_diff::State state_c2_with_noise(state_6d_with_noise);
  pose_diff::State state_diff_with_noise_c2_wrt_c1 = state_c2_with_noise.GetInverse();
  Eigen::Matrix4d transformation_matrix_with_noise_c2_wrt_c1 =
      state_diff_with_noise_c2_wrt_c1.GetTransformationMatrix();
  Eigen::Vector3d projection_diff_vector;
  error_from_noise.setZero();
  projection_diff_vector =
      pose_diff::Transform::Transform3DPoint(transformation_matrix_with_noise_c2_wrt_c1, c2_point_list[0]) -
      c1_point_list[0];
  error_from_noise = error_from_noise + (projection_diff_vector * projection_diff_vector.transpose()).diagonal();
  projection_diff_vector =
      pose_diff::Transform::Transform3DPoint(transformation_matrix_with_noise_c2_wrt_c1, c2_point_list[1]) -
      c1_point_list[1];
  error_from_noise = error_from_noise + (projection_diff_vector * projection_diff_vector.transpose()).diagonal();
  projection_diff_vector =
      pose_diff::Transform::Transform3DPoint(transformation_matrix_with_noise_c2_wrt_c1, c2_point_list[2]) -
      c1_point_list[2];
  error_from_noise = error_from_noise + (projection_diff_vector * projection_diff_vector.transpose()).diagonal();

  std::cout << "Error vector\n";
  print_vector(error_from_noise);
  std::cout << "\n\n";

  return 0;
}
