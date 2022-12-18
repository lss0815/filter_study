#ifndef FILTER_STUDY_POSE_DIFF_STATE_H
#define FILTER_STUDY_POSE_DIFF_STATE_H

#include <Eigen/Core>

#include "rotation.h"
#include "transform.h"

namespace pose_diff {
class State {
 public:
  explicit State(const Eigen::Matrix3d& rotation_matrix, Eigen::Vector3d translation)
      : translation_(std::move(translation)) {
    rotation_matrix_ = rotation_matrix;
  }
  explicit State(const Eigen::VectorXd& state_vector) {
    if (state_vector.size() == 6) {
      rotation_matrix_ = Rotation::AxisAngleToMatrix(state_vector.head(3));
      translation_ = state_vector.tail(3);
    }
  }
  explicit State(const Eigen::Matrix4d& transformation_matrix) {
    rotation_matrix_ << transformation_matrix.block<3, 3>(0, 0);
    translation_ << transformation_matrix.block<3, 1>(0, 3);
  }

  Eigen::VectorXd Get6DState() {
    Eigen::VectorXd state_6d(6);
    Eigen::Vector3d rotationXYZ = Rotation::MatrixToAxisAngle(rotation_matrix_);
    state_6d << rotationXYZ, translation_;
    return state_6d;
  };

  Eigen::Matrix4d GetTransformationMatrix() {
    Eigen::Matrix4d transformation_matrix;
    transformation_matrix.block<3, 3>(0, 0) = rotation_matrix_;
    transformation_matrix.block<3, 1>(0, 3) = translation_;
    transformation_matrix.block<1, 3>(3, 0).setZero();
    transformation_matrix(3, 3) = 1.;
    return transformation_matrix;
  }

  State GetInverse() {
    State inverse(rotation_matrix_.transpose(), -rotation_matrix_.transpose() * translation_);
    return inverse;
  }

 private:
  Eigen::Matrix3d rotation_matrix_;
  Eigen::Vector3d translation_;
};
}  // namespace pose_diff

#endif  //FILTER_STUDY_POSE_DIFF_STATE_H
