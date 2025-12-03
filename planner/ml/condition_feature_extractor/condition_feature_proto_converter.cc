#include "onboard/planner/ml/condition_feature_extractor/condition_feature_proto_converter.h"

#include <algorithm>  // for copy
#include <vector>     // for vector

namespace qcraft {
namespace planner {
namespace ml {

TrajectoryFeatureProto TrajectoryFeatureToProto(const TrajectoryFeature& feat) {
  TrajectoryFeatureProto pb;
  *pb.mutable_pos() = {feat.pos.begin(), feat.pos.end()};
  *pb.mutable_pos_diff() = {feat.pos_diff.begin(), feat.pos_diff.end()};
  *pb.mutable_speed() = {feat.speed.begin(), feat.speed.end()};
  *pb.mutable_yaw() = {feat.yaw.begin(), feat.yaw.end()};
  *pb.mutable_ts() = {feat.ts.begin(), feat.ts.end()};
  return pb;
}

LineSegmentsFeatureProto LineSegmentsFeatureToProto(
    const LineSegmentsFeature& feat) {
  LineSegmentsFeatureProto pb;
  *pb.mutable_node() = {feat.node.begin(), feat.node.end()};
  return pb;
}

PathBoundaryFeatureProto PathBoundaryFeatureToProto(
    const PathBoundaryFeature& feat) {
  PathBoundaryFeatureProto pb;
  *pb.mutable_left_path_boundary()->mutable_node() = {
      feat.left_path_boundary.node.begin(), feat.left_path_boundary.node.end()};
  *pb.mutable_right_path_boundary()->mutable_node() = {
      feat.right_path_boundary.node.begin(),
      feat.right_path_boundary.node.end()};
  *pb.mutable_smooth_ref_center_line()->mutable_node() = {
      feat.smooth_ref_center_line.node.begin(),
      feat.smooth_ref_center_line.node.end()};
  return pb;
}

}  // namespace ml
}  // namespace planner
}  // namespace qcraft
