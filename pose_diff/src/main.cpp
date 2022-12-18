#include "pose_diff/include/point_to_plane_icp.h"
#include "pose_diff/include/point_to_point_icp.h"

int main() {
  pose_diff::PointToPointICP point_to_point_icp;
  point_to_point_icp.Run();

  pose_diff::PointToPlaneICP point_to_plane_icp;
  point_to_plane_icp.Run();

  return 0;
}
