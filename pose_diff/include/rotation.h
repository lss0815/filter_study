#ifndef FILTER_STUDY_POSE_DIFF_ROTATION_H
#define FILTER_STUDY_POSE_DIFF_ROTATION_H

#include <cmath>
#include <iostream>

#include <Eigen/Dense>

namespace pose_diff {
// http://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
// Assuming R = R(yaw) * R(pitch) * R(roll)
class Rotation {
 public:
  static Eigen::Vector3d MatrixToRPY(const Eigen::Matrix3d& matrix) {
    Eigen::Vector3d rpy;
    if (!IsMatrixValid(matrix)) {
      std::cout << "Not valid rotation matrix\n";
      return rpy;
    }
    if (AreApproximatelyEqual(matrix(2, 0), 1.0, 1e-6)) {
      rpy(1) = M_PI_2;
      rpy(2) = 0.;
      rpy(0) = atan2(matrix(0, 1), matrix(0, 2));
    } else if (AreApproximatelyEqual(matrix(2, 0), -1.0, 1e-6)) {
      rpy(1) = -M_PI_2;
      rpy(2) = 0.;
      rpy(0) = atan2(-matrix(0, 1), -matrix(0, 2));
    } else {
      rpy(1) = -asin(matrix(2, 0));
      rpy(0) = atan2(matrix(2, 1) / cos(rpy(1)), matrix(2, 2) / cos(rpy(1)));
      rpy(2) = atan2(matrix(1, 0) / cos(rpy(1)), matrix(0, 0) / cos(rpy(1)));
    }
    return rpy;
  }

  static Eigen::Vector3d MatrixToAxisAngle(const Eigen::Matrix3d& matrix) {
    Eigen::Vector3d rot_vector;
    if (!IsMatrixValid(matrix)) {
      std::cout << "Not valid rotation matrix\n";
      return rot_vector;
    }
    Eigen::Vector3d axis_vector;
    if (matrix == matrix.transpose()) {
      Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(matrix);
      Eigen::Vector3cd eigen_value = eigen_solver.eigenvalues();
      for (int i = 0; i < 3; i++) {
        if (AreApproximatelyEqual(eigen_value(i).imag(), 0.) &&
            AreApproximatelyEqual(eigen_value(i).real(), 1.0, 1e-6)) {
          axis_vector = eigen_solver.eigenvectors().real().col(i).normalized();
          break;
        }
        if (i == 2) {
          std::cout << "Failed to get a eigenvector with eigenvalue 1\n";
          return rot_vector;
        }
      }
    } else {
      axis_vector(0) = matrix(2, 1) - matrix(1, 2);
      axis_vector(1) = matrix(0, 2) - matrix(2, 0);
      axis_vector(2) = matrix(1, 0) - matrix(0, 1);
      axis_vector.normalize();
    }
    double trace = matrix(0, 0) + matrix(1, 1) + matrix(2, 2);
    // right-handed expression uses [0, pi]
    double theta = acos((trace - 1.) / 2.);

    rot_vector = theta * axis_vector;
    if (AreApproximatelyEqual((AxisAngleToMatrix(rot_vector) - matrix).norm(), 0., 1e-6)) {
      return rot_vector;
    }
    return -1. * rot_vector;
  }

  static Eigen::Matrix3d AxisAngleToMatrix(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d matrix;
    double theta = vector.norm();
    Eigen::Vector3d axis = vector.normalized();
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    matrix(0, 0) = cos_th + axis(0) * axis(0) * (1. - cos_th);
    matrix(0, 1) = axis(0) * axis(1) * (1. - cos_th) - axis(2) * sin_th;
    matrix(0, 2) = axis(0) * axis(2) * (1. - cos_th) + axis(1) * sin_th;
    matrix(1, 0) = axis(0) * axis(1) * (1. - cos_th) + axis(2) * sin_th;
    matrix(1, 1) = cos_th + axis(1) * axis(1) * (1. - cos_th);
    matrix(1, 2) = axis(1) * axis(2) * (1. - cos_th) - axis(0) * sin_th;
    matrix(2, 0) = axis(0) * axis(2) * (1. - cos_th) - axis(1) * sin_th;
    matrix(2, 1) = axis(1) * axis(2) * (1. - cos_th) + axis(0) * sin_th;
    matrix(2, 2) = cos_th + axis(2) * axis(2) * (1. - cos_th);
    return matrix;
  }

 private:
  static bool IsMatrixValid(const Eigen::Matrix3d& matrix) {
    if (!(AreApproximatelyEqual(matrix.col(0).norm(), 1.0) && AreApproximatelyEqual(matrix.col(1).norm(), 1.0) &&
          AreApproximatelyEqual(matrix.col(2).norm(), 1.0))) {
      return false;
    }
    if (!(AreApproximatelyEqual(matrix.col(0).dot(matrix.col(1)), 0) &&
          AreApproximatelyEqual(matrix.col(1).dot(matrix.col(2)), 0) &&
          AreApproximatelyEqual(matrix.col(2).dot(matrix.col(0)), 0))) {
      return false;
    }
    return true;
  }

  static bool AreApproximatelyEqual(const double& a, const double& b, const double& maximum_diff = 1e-9) {
    return abs(a - b) < maximum_diff;
  }
};
}  // namespace pose_diff

#endif  //FILTER_STUDY_POSE_DIFF_ROTATION_H
