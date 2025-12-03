#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_LANE_SELECTION_FEATURE_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_LANE_SELECTION_FEATURE_H_

#include <string>
#include <vector>

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {

enum class LaneSelectionNetType {
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

inline LaneSelectionNetType ObjectTypeToLaneSelectionNetType(ObjectType ot) {
  switch (ot) {
    case OT_UNKNOWN_STATIC:
      return LaneSelectionNetType::AT_UNKNOWN_STATIC;
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
      return LaneSelectionNetType::AT_VEHICLE;
    case OT_MOTORCYCLIST:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return LaneSelectionNetType::AT_CYCLIST;
    case OT_PEDESTRIAN:
      return LaneSelectionNetType::AT_PEDESTRIAN;
    case OT_FOD:
      return LaneSelectionNetType::AT_FOD;
    case OT_UNKNOWN_MOVABLE:
      return LaneSelectionNetType::AT_UNKNOWN_MOVABLE;
    case OT_VEGETATION:
      return LaneSelectionNetType::AT_VEGETATION;
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
      return LaneSelectionNetType::AT_BARRIER;
    case OT_CONE:
      return LaneSelectionNetType::AT_CONE;
    case OT_WARNING_TRIANGLE:
      return LaneSelectionNetType::AT_WARNING_TRIANGLE;
  }
}

static constexpr struct LaneSelectionConfig {
  int max_pred_num = 16;
  double future_time_step = 0.1;  // s.
  int future_num = 30;
  int coord_num = 2;
  int channel_num = 5;
  int max_lane_seg_num = kFeatureV2MapSegmentNum;
  int max_other_objects_num = 33;
  int max_lc_num = 80;
  int max_lb_num = 40;
  int max_crosswalk_num = 8;
  int max_dp_num = 80;
  double length_scale = 10.0;
  double width_scale = 3.0;
  double scan_box_front_dist = 150.0;
  double scan_box_back_dist = 30.0;
  double scan_box_half_width = 60.0;
} kLaneSelectionConfig;  // NOLINT

struct LaneSelectionObjectCommonFeatures {
  std::vector<float> sl_pos;
  std::vector<float> sl_speed;
  std::vector<float> yaw_diff_lane_path;
  std::vector<float> length_width;
  std::vector<float> sl_shape;
  std::vector<float> yaw;
  std::vector<float> mask;
  float type;

  // For dist calculation
  std::vector<float> pos;
};

struct LaneSelectionAgentFeatures {
  // common features
  std::vector<float> sl_pos;
  std::vector<float> sl_speed;
  std::vector<float> yaw_diff_lane_path;
  std::vector<float> length_width;
  std::vector<float> sl_shape;
  std::vector<float> mask;
  float type;

  // agent specific features
  float l_leap;
  std::vector<float> sl_acc;
  std::vector<float> yaw_diff_self_leap;

  std::vector<float> yaw_diff_lane_path_leap;

  std::vector<float> dist_to_left_lane_boundary;
  float dist_to_left_lane_boundary_leap;

  std::vector<float> dist_to_right_lane_boundary;
  float dist_to_right_lane_boundary_leap;

  std::vector<float> shape_dist_to_left_lane_boundary;
  std::vector<float> shape_dist_to_right_lane_boundary;

  std::vector<float> lane_width;
};

struct LaneSelectionObjectFeatures {
  std::vector<float> sl_pos;
  std::vector<float> sl_speed;
  std::vector<float> yaw_diff_lane_path;
  std::vector<float> length_width;
  std::vector<float> sl_shape;
  float type;

  std::vector<float> rel_sl_pos_agent;
  std::vector<float> rel_sl_speed_agent;
  std::vector<float> yaw_diff_agent;
  std::vector<float> rel_dist_agent;
};

struct LaneChangeSLPolylineFeatures {
  mapping::ElementId id = mapping::kInvalidElementId;
  // Absolute features in Agent's coordinate frame.
  std::vector<float> sl_seg;
  std::vector<float> seg_len;
  std::vector<float> dist;
  std::vector<float> nearest_to_end_dist;
  std::vector<float> yaw_diff;
  std::vector<float> type;
  std::vector<float> light;
  std::vector<float> mask;

  // For region box selection
  std::vector<float> seg;
};

struct LaneSelectionFeature {
  std::string agent_id;

  // Objects and agent features
  LaneSelectionAgentFeatures agent_sl_feature;
  std::vector<LaneSelectionObjectFeatures> objs_sl_features;

  // Map features
  std::vector<LaneChangeSLPolylineFeatures> lc_sl_features;
  std::vector<LaneChangeSLPolylineFeatures> lb_sl_features;
  std::vector<LaneChangeSLPolylineFeatures> cw_sl_features;

  // Map masks
  std::vector<float> lc_masks;
  std::vector<float> lb_masks;
  std::vector<float> cw_masks;

  // lane_center_xy_seg
  std::vector<float> lane_path_center_xy_seg;

  // lane_path_boundary_sl_pos
  std::vector<float> left_lane_path_boundary_sl_pos;
  std::vector<float> right_lane_path_boundary_sl_pos;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_LANE_SELECTION_FEATURE_H_
