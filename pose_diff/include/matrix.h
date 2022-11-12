#ifndef FILTER_STUDY_POSE_DIFF_MATRIX_H
#define FILTER_STUDY_POSE_DIFF_MATRIX_H

#include <Eigen/Core>

namespace pose_diff {
class Matrix {
 public:
  static Eigen::Matrix3d GetSkewSymmetricMatrix(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d skew_symmetric_matrix;
    skew_symmetric_matrix << 0., -vector(2), vector(1), vector(2), 0., -vector(0), -vector(1), vector(0), 0.;
    return skew_symmetric_matrix;
  }
};
}  // namespace pose_diff

#endif  //FILTER_STUDY_POSE_DIFF_MATRIX_H
