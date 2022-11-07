#ifndef FILTER_STUDY_POSE_DIFF_ROTATION_H
#define FILTER_STUDY_POSE_DIFF_ROTATION_H

#include <cmath>

#include <Eigen/Core>

namespace pose_diff {
// http://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
// Assuming R = R(yaw) * R(pitch) * R(roll)
class Rotation {
 public:
  static Eigen::Vector3d MatrixToRPY(const Eigen::Matrix3d& matrix) {
    Eigen::Vector3d rpy;
    if (!IsMatrixValid(matrix)) {
      return rpy;
    }
    if (AreApproximatelyEqual(matrix(2, 0), 1.0, 1e-6)) {
      rpy(1) = M_PI_2;
      rpy(2) = 0.;
      rpy(0) = atan2(matrix(0,1), matrix(0,2));
    } else if (AreApproximatelyEqual(matrix(2, 0), -1.0, 1e-6)) {
      rpy(1) = -M_PI_2;
      rpy(2) = 0.;
      rpy(0) = atan2(-matrix(0,1), -matrix(0,2));
    } else {
      rpy(1) = -asin(matrix(2, 0));
      rpy(0) = atan2(matrix(2, 1) / cos(rpy(1)),
                     matrix(2, 2) / cos(rpy(1)));
      rpy(2) = atan2(matrix(1, 0) / cos(rpy(1)),
                     matrix(0, 0) / cos(rpy(1)));
    }
    return rpy;
  }
 private:
  static bool IsMatrixValid(const Eigen::Matrix3d& matrix) {
    if (!(AreApproximatelyEqual(matrix.col(0).norm(), 1.0) &&
          AreApproximatelyEqual(matrix.col(1).norm(), 1.0) &&
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
