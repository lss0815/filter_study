#include "pose_diff/include/point_to_plane_icp.h"

namespace pose_diff {
void PointToPlaneICP::Run() {
  Eigen::Matrix4d pose_matrix;
  // T_c1_wrt_c2
  pose_matrix << -1, 0, 0, -0.1, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;
  pose_diff::State state(pose_matrix);

  Eigen::VectorXd state_6d = state.Get6DState();

  std::cout << "state of C1 is identity\n\n\n";

  std::cout << "state of C2\n" << state_6d.transpose() << "\n\n";
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

  std::cout << "state of C2 with noise\n" << state_6d_with_noise.transpose() << "\n\n";

  const int kPointNum = 3;
  const double kPointPosStdDev = 5.0;
  std::normal_distribution<double> point_pos_gaussian(0, kPointPosStdDev);
  std::vector<Eigen::Vector3d> c1_point_list(kPointNum);
  for (int i = 0; i < kPointNum; i++) {
    c1_point_list[i] << point_pos_gaussian(random_engine), point_pos_gaussian(random_engine),
        point_pos_gaussian(random_engine);
  }
  pose_diff::State state_diff_c2_wrt_c1 = state.GetInverse();
  Eigen::Matrix4d transformation_matrix_c2_wrt_c1 = state_diff_c2_wrt_c1.GetTransformationMatrix();
  std::vector<Eigen::Vector3d> c2_point_list(kPointNum);
  for (int i = 0; i < kPointNum; i++) {
    c2_point_list[i] = pose_diff::Transform::Transform3DPoint(transformation_matrix_c2_wrt_c1, c1_point_list[i]);
  }

  const int kPlaneNum = 10;
  std::vector<Eigen::Vector3d> c1_plane_point_list;
  std::vector<Eigen::Vector3d> c1_normal_vector_list;
  std::vector<Eigen::Vector3d> c2_plane_point_list;
  std::cout << "\n\nPlane observations, normal vector / point at c1\n";
  for (int i = 0; i < kPlaneNum; i++) {
    Eigen::Vector3d normal_vector;
    Eigen::Vector3d c1_plane_point;
    Eigen::Vector3d c2_plane_point;
    GeneratePlaneAndSamplePoints(normal_vector, c1_plane_point, c2_plane_point);
    std::cout << normal_vector.transpose() << " / " << c1_plane_point.transpose() << "\n";
    c1_normal_vector_list.emplace_back(std::move(normal_vector));
    c1_plane_point_list.emplace_back(std::move(c1_plane_point));
    c2_plane_point_list.emplace_back(std::move(c2_plane_point));
  }

  std::cout << "\n\nPlane observations, point at c2\n";
  for (int i = 0; i < kPlaneNum; i++) {
    c2_plane_point_list[i] =
        pose_diff::Transform::Transform3DPoint(transformation_matrix_c2_wrt_c1, c2_plane_point_list[i]);
    std::cout << c2_plane_point_list[i].transpose() << "\n";
  }

  Eigen::VectorXd error_vector(3 * kPointNum + kPlaneNum);
  pose_diff::State state_c2_with_noise(state_6d_with_noise);
  pose_diff::State state_diff_with_noise_c2_wrt_c1 = state_c2_with_noise.GetInverse();
  Eigen::Matrix4d transformation_matrix_with_noise_c2_wrt_c1 =
      state_diff_with_noise_c2_wrt_c1.GetTransformationMatrix();
  for (int i = 0; i < kPointNum; i++) {
    // e = Rp' + t - p
    error_vector.segment<3>(3 * i) =
        pose_diff::Transform::Transform3DPoint(transformation_matrix_with_noise_c2_wrt_c1, c2_point_list[i]) -
        c1_point_list[i];
  }
  for (int i = 0; i < kPlaneNum; i++) {
    error_vector(kPointNum * 3 + i) = c1_normal_vector_list[i].dot(
        pose_diff::Transform::Transform3DPoint(transformation_matrix_with_noise_c2_wrt_c1, c2_plane_point_list[i]) -
        c1_plane_point_list[i]);
  }
  std::cout << "Error vector\n" << error_vector.transpose() << "\n";
  std::cout << "Error vector norm\n";
  std::cout << error_vector.norm();
  std::cout << "\n\n";

  Eigen::MatrixXd jacobian_matrix(3 * kPointNum + kPlaneNum, 6);
  Eigen::Matrix3d rotation_matrix = transformation_matrix_with_noise_c2_wrt_c1.block<3, 3>(0, 0);
  for (int i = 0; i < kPointNum; i++) {
    // jacobian matrix : [ -skew_symm(R * p) I_3x3]
    // derivation : TBD
    Eigen::Vector3d rotated_point = rotation_matrix * c2_point_list[i];
    jacobian_matrix.block<3, 3>(3 * i, 0) = -pose_diff::Matrix::GetSkewSymmetricMatrix(rotated_point);
    jacobian_matrix.block<3, 3>(3 * i, 3).setIdentity();
  }
  for (int i = 0; i < kPlaneNum; i++) {
    Eigen::Vector3d rotated_point = rotation_matrix * c2_plane_point_list[i];
    jacobian_matrix(kPointNum * 3 + i, 0) =
        -c1_normal_vector_list[i](1) * rotated_point(2) + c1_normal_vector_list[i](2) * rotated_point(1);
    jacobian_matrix(kPointNum * 3 + i, 1) =
        c1_normal_vector_list[i](0) * rotated_point(2) - c1_normal_vector_list[i](2) * rotated_point(0);
    jacobian_matrix(kPointNum * 3 + i, 2) =
        -c1_normal_vector_list[i](0) * rotated_point(1) + c1_normal_vector_list[i](1) * rotated_point(0);
    jacobian_matrix.block<1, 3>(kPointNum * 3 + i, 3) = c1_normal_vector_list[i].transpose();
  }
  std::cout << "0th iter jacobian\n" << jacobian_matrix << "\n\n";

  // e(x + dx) (\approx) e + H dx = 0
  // dx = - H.inv e
  // derivation: TBD
  Eigen::VectorXd current_state = state_diff_with_noise_c2_wrt_c1.Get6DState();
  const double kInitialErrorNorm = error_vector.norm();
  const int kMaxIter = 100;
  int iter_count = 0;
  while (iter_count++ < kMaxIter) {
    std::cout << "Iter " << iter_count << "\n";
    Eigen::VectorXd state_diff = -pose_diff::Matrix::GetPseudoInverseMatrix(jacobian_matrix) * error_vector;
    std::cout << "current state\n" << current_state.transpose() << "\n";
    std::cout << "dx vector\n" << state_diff.transpose() << "\n";
    Eigen::VectorXd next_state = current_state + state_diff;
    next_state.head(3) =
        pose_diff::Rotation::MatrixToAxisAngle(pose_diff::Rotation::AxisAngleToMatrix(state_diff.head(3)) *
                                               pose_diff::Rotation::AxisAngleToMatrix(current_state.head(3)));
    std::cout << "next state\n" << next_state.transpose() << "\n";
    Eigen::Matrix4d transformation_matrix = pose_diff::State(next_state).GetTransformationMatrix();
    for (int i = 0; i < kPointNum; i++) {
      // e = Rp' + t - p
      error_vector.segment<3>(3 * i) =
          pose_diff::Transform::Transform3DPoint(transformation_matrix, c2_point_list[i]) - c1_point_list[i];
    }
    for (int i = 0; i < kPlaneNum; i++) {
      error_vector(kPointNum * 3 + i) = c1_normal_vector_list[i].dot(
          pose_diff::Transform::Transform3DPoint(transformation_matrix, c2_plane_point_list[i]) -
          c1_plane_point_list[i]);
    }
    std::cout << "Error vector\n" << error_vector.transpose() << "\n";
    std::cout << "Error vector norm\n" << error_vector.norm() << "\n\n";

    double error_norm = error_vector.norm();
    if (error_norm < kInitialErrorNorm * 1e-10 || error_norm < 1e-10) {
      std::cout << "Succeeded to optimize. error: " << error_norm << ", initial error:" << kInitialErrorNorm << "\n";
      break;
    }

    rotation_matrix = pose_diff::Rotation::AxisAngleToMatrix(next_state.head(3));
    for (int i = 0; i < kPointNum; i++) {
      // jacobian matrix : [ -skew_symm(R * p) I_3x3]
      // derivation : TBD
      Eigen::Vector3d rotated_point = rotation_matrix * c2_point_list[i];
      jacobian_matrix.block<3, 3>(3 * i, 0) = -pose_diff::Matrix::GetSkewSymmetricMatrix(rotated_point);
      jacobian_matrix.block<3, 3>(3 * i, 3).setIdentity();
    }
    for (int i = 0; i < kPlaneNum; i++) {
      Eigen::Vector3d rotated_point = rotation_matrix * c2_plane_point_list[i];
      jacobian_matrix(kPointNum * 3 + i, 0) =
          -c1_normal_vector_list[i](1) * rotated_point(2) + c1_normal_vector_list[i](2) * rotated_point(1);
      jacobian_matrix(kPointNum * 3 + i, 1) =
          c1_normal_vector_list[i](0) * rotated_point(2) - c1_normal_vector_list[i](2) * rotated_point(0);
      jacobian_matrix(kPointNum * 3 + i, 2) =
          -c1_normal_vector_list[i](0) * rotated_point(1) + c1_normal_vector_list[i](1) * rotated_point(0);
      jacobian_matrix.block<1, 3>(kPointNum * 3 + i, 3) = c1_normal_vector_list[i].transpose();
    }

    current_state = next_state;
  }
  if (iter_count > kMaxIter) {
    std::cout << "Failed to optimize. error: " << error_vector.norm() << ", initial error:" << kInitialErrorNorm
              << "\n";
  }
}

void PointToPlaneICP::GeneratePlaneAndSamplePoints(Eigen::Vector3d& normal_vector, Eigen::Vector3d& point1,
                                                   Eigen::Vector3d& point2) {
  static double kCenterPointDev = 10.0;
  static double kNearPointDev = 1.0;
  static std::default_random_engine random_engine(time(nullptr));
  static std::normal_distribution<double> center_point_gaussian(0, kCenterPointDev);
  static std::normal_distribution<double> near_point_gaussian(0, kNearPointDev);

  Eigen::Vector3d point3;
  while (true) {
    point1 << center_point_gaussian(random_engine), center_point_gaussian(random_engine),
        center_point_gaussian(random_engine);
    point2 << point1(0) + near_point_gaussian(random_engine), point1(1) + near_point_gaussian(random_engine),
        point1(2) + near_point_gaussian(random_engine);
    point3 << point1(0) + near_point_gaussian(random_engine), point1(1) + near_point_gaussian(random_engine),
        point1(2) + near_point_gaussian(random_engine);

    normal_vector = (point1 - point2).cross(point1 - point3);
    if (normal_vector.norm() > 1e-6) {
      normal_vector.normalize();
      break;
    }
  }
}

}  // namespace pose_diff
