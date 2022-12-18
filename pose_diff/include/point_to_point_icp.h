#ifndef FILTER_STUDY_POSE_DIFF_POINT_TO_POINT_ICP_H
#define FILTER_STUDY_POSE_DIFF_POINT_TO_POINT_ICP_H

#include "pose_diff/include/core/matrix.h"
#include "pose_diff/include/core/rotation.h"
#include "pose_diff/include/core/state.h"
#include "pose_diff/include/core/transform.h"

#include <ctime>
#include <iostream>
#include <random>

#include <Eigen/Core>

namespace pose_diff {
class PointToPointICP {
 public:
  void Run();
};
}  // namespace pose_diff

#endif  //FILTER_STUDY_POSE_DIFF_POINT_TO_POINT_ICP_H
