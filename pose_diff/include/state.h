#ifndef FILTER_STUDY_POSE_DIFF_STATE_H
#define FILTER_STUDY_POSE_DIFF_STATE_H

#include <Eigen/Core>

#include "pose_diff/include/rotation.h"

namespace pose_diff {
class State {
 public:
  explicit State(const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& translation)
      : translation_(translation) {
    rotation_matrix_ = rotation_matrix;
  }
  explicit State(const Eigen::Matrix4d& transformation_matrix) {
    rotation_matrix_ << transformation_matrix.block<3, 3>(0, 0);
    translation_ << transformation_matrix.block<3, 1>(0, 3);
  }

  Eigen::VectorXd Get6DState() {
    Eigen::VectorXd state_6d(6);
    Eigen::Vector3d rotationXYZ = Rotation::MatrixToRPY(rotation_matrix_);
    state_6d << rotationXYZ, translation_;
    return state_6d;
  };
 private:
  Eigen::Matrix3d rotation_matrix_;
  Eigen::Vector3d translation_;
};
}  // namespace pose_diff

#endif  //FILTER_STUDY_POSE_DIFF_STATE_H
