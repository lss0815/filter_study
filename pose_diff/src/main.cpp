#include "pose_diff/include/state.h"

#include <iostream>

#include <Eigen/Core>

int main(){
  Eigen::Matrix4d pose_matrix;
  pose_matrix << -1 , 0, 0, -0.1,
                  0, 1, 0, 0,
                  0, 0, -1, 0,
                  0, 0, 0, 1;
  pose_diff::State state(pose_matrix);

  Eigen::VectorXd state_6d = state.Get6DState();

  for (int i=0; i<6; i++) {
    std::cout << state_6d(i) << " ";
  }
  std::cout << "\n";

  return 0;
}
