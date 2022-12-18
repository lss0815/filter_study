#ifndef FILTER_STUDY_POSE_DIFF_POINT_TO_PLANE_ICP_H
#define FILTER_STUDY_POSE_DIFF_POINT_TO_PLANE_ICP_H

#include "pose_diff/include/core/matrix.h"
#include "pose_diff/include/core/rotation.h"
#include "pose_diff/include/core/state.h"
#include "pose_diff/include/core/transform.h"

#include <ctime>
#include <iostream>
#include <random>

#include <Eigen/Core>

namespace pose_diff {
class PointToPlaneICP {
 public:
  void Run();

 private:
  void GeneratePlaneAndSamplePoints(Eigen::Vector3d& normal_vector, Eigen::Vector3d& point1, Eigen::Vector3d& point2);
};
}  // namespace pose_diff

#endif  //FILTER_STUDY_POSE_DIFF_POINT_TO_PLANE_ICP_H
