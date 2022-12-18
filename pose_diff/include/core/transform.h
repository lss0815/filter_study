#ifndef FILTER_STUDY_POSE_DIFF_TRANSFORM_H
#define FILTER_STUDY_POSE_DIFF_TRANSFORM_H

#include <Eigen/Core>

namespace pose_diff {
class Transform {
 public:
  static Eigen::Vector3d Transform3DPoint(const Eigen::Matrix4d& transform_matrix, const Eigen::Vector3d& point) {
    Eigen::Vector3d transformed_point =
        Transform3DPoint(transform_matrix.block<3, 3>(0, 0), transform_matrix.block<3, 1>(0, 3), point);
    return transformed_point;
  }

  static Eigen::Vector3d Transform3DPoint(const Eigen::Matrix3d& rot_matrix, const Eigen::Vector3d& trans_vector,
                                          const Eigen::Vector3d& point) {
    Eigen::Vector3d transformed_point = rot_matrix * point + trans_vector;
    return transformed_point;
  }
};
}  // namespace pose_diff

#endif  //FILTER_STUDY_POSE_DIFF_TRANSFORM_H
