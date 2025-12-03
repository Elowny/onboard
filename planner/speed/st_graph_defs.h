#ifndef ONBOARD_PLANNER_SPEED_ST_GRAPH_DEFS_H_
#define ONBOARD_PLANNER_SPEED_ST_GRAPH_DEFS_H_
#include <string>
#include <vector>

#include "onboard/math/geometry/box2d.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
namespace qcraft::planner {
struct StDistancePoint {
  double t = 0.0;
  double path_s = 0.0;
  double distance = 0.0;
  double relative_v = 0.0;
};

struct CloseSpaceTimeObject {
  std::vector<StDistancePoint> st_distance_points;
  StBoundaryProto::ObjectType object_type;
  std::string id;
  Box2d box;
};

struct DistanceInfo {
  double s = 0.0;
  double dist = 0.0;
  std::string id;
  std::string info;
};
}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_SPEED_ST_GRAPH_DEFS_H_
