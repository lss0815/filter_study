#include "pose_diff/include/matrix.h"
#include "pose_diff/include/rotation.h"
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

  auto print_matrix = [](const Eigen::MatrixXd& matrix) {
    for (int i = 0; i < matrix.rows(); i++) {
      for (int j = 0; j < matrix.cols(); j++) {
        std::cout << matrix(i, j) << " ";
      }
      std::cout << "\n";
    }
  };

  std::cout << "state of C1 is identity\n\n\n";

  std::cout << "state of C2\n";
  print_vector(state_6d);
  std::cout << "\n\n";

  const double KRotStdDev = 0.5;
  const double kTransStdDev = 0.3;
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

  const int kPointNum = 20;
  const double kPointPosStdDev = 5.0;
  std::normal_distribution<double> point_pos_gaussian(0, kPointPosStdDev);
  std::vector<Eigen::Vector3d> c1_point_list(kPointNum);
  for (int i = 0; i < kPointNum; i++) {
    c1_point_list[i] << point_pos_gaussian(random_engine), point_pos_gaussian(random_engine),
        point_pos_gaussian(random_engine);
  }

  std::cout << "Points at C1\n";
  for (int i = 0; i < kPointNum; i++) {
    print_vector(c1_point_list[i]);
  }
  pose_diff::State state_diff_c2_wrt_c1 = state.GetInverse();
  Eigen::Matrix4d transformation_matrix_c2_wrt_c1 = state_diff_c2_wrt_c1.GetTransformationMatrix();
  std::vector<Eigen::Vector3d> c2_point_list(kPointNum);
  for (int i = 0; i < kPointNum; i++) {
    c2_point_list[i] = pose_diff::Transform::Transform3DPoint(transformation_matrix_c2_wrt_c1, c1_point_list[i]);
  }
  std::cout << "\n\n";

  std::cout << "Points at C2\n";
  for (int i = 0; i < kPointNum; i++) {
    print_vector(c2_point_list[i]);
  }
  std::cout << "\n\n";

  Eigen::VectorXd error_vector(3 * kPointNum);
  pose_diff::State state_c2_with_noise(state_6d_with_noise);
  pose_diff::State state_diff_with_noise_c2_wrt_c1 = state_c2_with_noise.GetInverse();
  Eigen::Matrix4d transformation_matrix_with_noise_c2_wrt_c1 =
      state_diff_with_noise_c2_wrt_c1.GetTransformationMatrix();
  for (int i = 0; i < kPointNum; i++) {
    // e = Rp' + t - p
    error_vector.segment<3>(3 * i) =
        pose_diff::Transform::Transform3DPoint(transformation_matrix_with_noise_c2_wrt_c1, c2_point_list[0]) -
        c1_point_list[0];
  }
  std::cout << "Error vector\n";
  print_vector(error_vector);
  std::cout << "Error vector mean\n";
  std::cout << error_vector.mean();
  std::cout << "\n\n";

  Eigen::MatrixXd jacobian_matrix(3 * kPointNum, 6);
  Eigen::Matrix3d rotation_matrix = transformation_matrix_with_noise_c2_wrt_c1.block<3, 3>(0, 0);
  for (int i = 0; i < kPointNum; i++) {
    // jacobian matrix : [ -skew_symm(R * p) I_3x3]
    // derivation : https://lss0815.tistory.com/10
    Eigen::Vector3d rotated_point = rotation_matrix * c2_point_list[i];
    jacobian_matrix.block<3, 3>(3 * i, 0) = -pose_diff::Matrix::GetSkewSymmetricMatrix(rotated_point);
    jacobian_matrix.block<3, 3>(3 * i, 3).setIdentity();
  }
  std::cout << "0th iter jacobian\n";
  print_matrix(jacobian_matrix);
  std::cout << "\n\n";

  // e(x + dx) (\approx) e + H dx = 0
  // dx = - H.inv e
  Eigen::VectorXd state_diff = -pose_diff::Matrix::GetPseudoInverseMatrix(jacobian_matrix) * error_vector;
  Eigen::VectorXd current_state = state_diff_with_noise_c2_wrt_c1.Get6DState();
  Eigen::VectorXd next_state = current_state + state_diff;
  next_state.head(3) =
      pose_diff::Rotation::MatrixToAxisAngle(pose_diff::Rotation::AxisAngleToMatrix(current_state.head(3)) *
                                             pose_diff::Rotation::AxisAngleToMatrix(state_diff.head(3)));
  Eigen::Matrix4d transformation_matrix = pose_diff::State(next_state).GetTransformationMatrix();
  for (int i = 0; i < kPointNum; i++) {
    // e = Rp' + t - p
    error_vector.segment<3>(3 * i) =
        pose_diff::Transform::Transform3DPoint(transformation_matrix, c2_point_list[0]) - c1_point_list[0];
  }
  std::cout << "Iter 1\n";
  std::cout << "Error vector\n";
  print_vector(error_vector);
  std::cout << "Error vector mean\n";
  std::cout << error_vector.mean();
  std::cout << "\n\n";

  return 0;
}
