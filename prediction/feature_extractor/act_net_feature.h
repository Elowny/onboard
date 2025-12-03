#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_FEATURE_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_FEATURE_H_

#include <string>
#include <vector>

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/prediction.pb.h"
namespace qcraft {
namespace prediction {
enum class ActNetType {
  AT_UNSPECIFIED = 0,        // NOLINT
  AT_UNKNOWN_STATIC = 1,     // NOLINT
  AT_VEHICLE = 2,            // NOLINT
  AT_CYCLIST = 3,            // NOLINT
  AT_PEDESTRIAN = 4,         // NOLINT
  AT_FOD = 5,                // NOLINT
  AT_UNKNOWN_MOVABLE = 6,    // NOLINT
  AT_VEGETATION = 7,         // NOLINT
  AT_BARRIER = 8,            // NOLINT
  AT_CONE = 9,               // NOLINT
  AT_WARNING_TRIANGLE = 10,  // NOLINT
};

inline ActNetType ObjectTypeToActNetType(ObjectType ot) {
  switch (ot) {
    case OT_UNKNOWN_STATIC:
      return ActNetType::AT_UNKNOWN_STATIC;
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
      return ActNetType::AT_VEHICLE;
    case OT_MOTORCYCLIST:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return ActNetType::AT_CYCLIST;
    case OT_PEDESTRIAN:
      return ActNetType::AT_PEDESTRIAN;
    case OT_FOD:
      return ActNetType::AT_FOD;
    case OT_UNKNOWN_MOVABLE:
      return ActNetType::AT_UNKNOWN_MOVABLE;
    case OT_VEGETATION:
      return ActNetType::AT_VEGETATION;
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
      return ActNetType::AT_BARRIER;
    case OT_CONE:
      return ActNetType::AT_CONE;
    case OT_WARNING_TRIANGLE:
      return ActNetType::AT_WARNING_TRIANGLE;
  }
}
inline constexpr struct ActNetConfig {
  int max_pred_num = 64;
  double future_time_step = 0.1;  // s.
  int future_num = 100;
  int coord_num = 2;
  int max_lane_seg_num = kFeatureV2MapSegmentNum;
  int max_other_objects_num = 33;
  int max_lc_num = 160;
  int max_solid_lb_num = 80;
  int max_broken_lb_num = 120;
  int max_crosswalk_num = 16;
  double av_map_radius = 210;
  double av_map_offset = 60;
  double scan_box_front = 150.0;
  double scan_box_back = 30.0;
  double scan_box_half_width = 60.0;
  double max_path_length = 80.0;
  int max_path_num = 11;
  double max_cutin_time = 5.0;
  double cutin_distance = 1.1;
} kActNetConfig;

struct ActNetObjectAbsoluteFeature {
  std::vector<float> pos;
  std::vector<float> pos_diff;
  std::vector<float> speed;
  std::vector<float> yaw;
  std::vector<float> shape;
  std::vector<float> ts;
  std::vector<float> mask;
  float type;
  // length & width
  std::vector<float> lw;
};

struct ActNetObjectRelFeature {
  // Relative features (w.r.t each history point of agent) in Agent's coordinate
  // frame.
  std::vector<float> rel_pos;
  std::vector<float> rel_dist;
  std::vector<float> rel_yaw;
  std::vector<float> yaw_diff;
  std::vector<float> rel_speed;
  std::vector<float> rel_shape;
  std::vector<float> rel_mask;
};

struct ActNetObjectFeature {
  ObjectIDType id;
  // Absolute features in Agent's coordinate frame.
  ActNetObjectAbsoluteFeature abs_feat;
  ActNetObjectRelFeature rel_feat;
};

struct ActNetPolylineFeature {
  mapping::ElementId id = mapping::kInvalidElementId;
  // Absolute features in Agent's coordinate frame.
  std::vector<float> seg;
  std::vector<float> unit_seg_vec;
  std::vector<float> seg_len;
  std::vector<float> unit_tangent;
  std::vector<float> dist;
  std::vector<float> unit_dist_vec;
  std::vector<float> nearest_to_end_dist;
  std::vector<float> yaw_diff;
  std::vector<float> type;
  std::vector<float> light;
  std::vector<float> mask;
  std::vector<float> speed_limit_kph;
  std::vector<float> boundary_type;
  std::vector<float> lb_seg;
  std::vector<float> seg_width;
};

struct ActNetCutInInfo {
  std::vector<float> path_xy;
  std::vector<float> headings;
  std::vector<float> dist;
  std::vector<float> unit_dist_vec;
  bool already_on_path;
};

struct ActNetFeature {
  std::string agent_id;
  Vec2d ref_position;
  float rot_rad;
  std::vector<float> stop_time_info;
  ActNetObjectAbsoluteFeature agent_feature;
  std::vector<ActNetObjectFeature> context_obj_features;
  std::vector<float> context_obj_masks;
  ActNetCutInInfo cutin_info;
  std::vector<ActNetPolylineFeature> lc_features;
  std::vector<ActNetPolylineFeature> solid_lb_features;
  std::vector<ActNetPolylineFeature> broken_lb_features;
  std::vector<ActNetPolylineFeature> cw_features;
  std::vector<float> lc_masks;
  std::vector<float> solid_lb_masks;
  std::vector<float> broken_lb_masks;
  std::vector<float> cw_masks;
};

struct AgentCoordTransform {
  Vec2d orig;
  double rot_rad;
  double rot_sin;
  double rot_cos;
};

struct AvMapCache {
  std::vector<const mapping::LaneInfo*> lane_centers;
  std::vector<const mapping::LaneBoundaryInfo*> solid_lane_boundarys;
  std::vector<const mapping::LaneBoundaryInfo*> broken_lane_boundarys;
  std::vector<const mapping::CrosswalkInfo*> cross_walks;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_FEATURE_H_
