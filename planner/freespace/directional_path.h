#ifndef ONBOARD_PLANNER_FREESPACE_DIRECTIONAL_PATH_H_
#define ONBOARD_PLANNER_FREESPACE_DIRECTIONAL_PATH_H_

#include <utility>
#include <vector>

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"

namespace qcraft {
namespace planner {

struct DirectionalPath {
  // Heading of a path point is the moving direction.
  DiscretizedPath path;
  // How should AV follow the path. False for reverse driving.
  bool forward = true;

  void FromProto(const DirectionalPathProto& proto) {
    std::vector<PathPoint> path_points(proto.path().begin(),
                                       proto.path().end());
    path = DiscretizedPath(std::move(path_points));
    forward = proto.forward();
  }
  void ToProto(DirectionalPathProto* proto) const {
    *proto->mutable_path() = {path.begin(), path.end()};
    proto->set_forward(forward);
  }
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_DIRECTIONAL_PATH_H_
