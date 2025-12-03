#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_CUTIN_SL_FEATURE_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_CUTIN_SL_FEATURE_H_

#include <string>
#include <vector>

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
enum class CutinSLNetType {
  kAtUnspecified = 0,
  kAtUnknownStatic = 1,
  kAtVehicle = 2,
  kAtCyclist = 3,
  kAtPedestrian = 4,
  kAtFod = 5,
  kAtUnknownMovable = 6,
  kAtVegetation = 7,
  kAtBarrier = 8,
  kAtCone = 9,
  kAtWarningTriangle = 10,
};

enum class CrossType { kNotcrossed = 0, kCrossed = 1, kUnknown = 2 };

inline CutinSLNetType ObjectTypeToCutinSLNetType(ObjectType ot) {
  switch (ot) {
    case OT_UNKNOWN_STATIC:
      return CutinSLNetType::kAtUnknownStatic;
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
      return CutinSLNetType::kAtVehicle;
    case OT_MOTORCYCLIST:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return CutinSLNetType::kAtCyclist;
    case OT_PEDESTRIAN:
      return CutinSLNetType::kAtPedestrian;
    case OT_FOD:
      return CutinSLNetType::kAtFod;
    case OT_UNKNOWN_MOVABLE:
      return CutinSLNetType::kAtUnknownMovable;
    case OT_VEGETATION:
      return CutinSLNetType::kAtVegetation;
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
      return CutinSLNetType::kAtBarrier;
    case OT_CONE:
      return CutinSLNetType::kAtCone;
    case OT_WARNING_TRIANGLE:
      return CutinSLNetType::kAtWarningTriangle;
  }
}

struct CutinSLObjectAbsoluteFeature {
  std::vector<float> sl_pos;
  std::vector<float> sl_speed;
  std::vector<float> yaw_diff_dp;
  std::vector<float> length_width;
  std::vector<float> sl_shape;
  std::vector<float> yaw;
  std::vector<float> mask;
  float type;

  // For dist calculation
  std::vector<float> pos;
  std::vector<float> agent_center_dist_to_av_left_lane_boundary;
  std::vector<float> agent_center_dist_to_av_right_lane_boundary;
  std::vector<float> agent_corners_dist_to_av_left_lane_boundary;
  std::vector<float> agent_corners_dist_to_av_right_lane_boundary;
};

struct CutinSLObjectFeatures {
  std::vector<float> sl_pos;
  std::vector<float> sl_speed;
  std::vector<float> yaw_diff_dp;
  std::vector<float> length_width;
  std::vector<float> sl_shape;
  float type;

  std::vector<float> rel_sl_pos_agent;
  std::vector<float> rel_sl_speed_agent;
  std::vector<float> yaw_diff_agent;
  std::vector<float> rel_dist_agent;
  std::vector<float> obj_center_dist_to_av_left_lane_boundary;
  std::vector<float> obj_center_dist_to_av_right_lane_boundary;
  std::vector<float> obj_corners_dist_to_av_left_lane_boundary;
  std::vector<float> obj_corners_dist_to_av_right_lane_boundary;
};

struct CutinSLPolylineFeatures {
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

static constexpr struct CutinSLConfig {
  int max_pred_num = 16;
  double future_time_step = 0.1;  // s.
  int future_num = 30;
  int coord_num = 2;
  int channel_num = 5;
  int max_lane_seg_num = kFeatureV2MapSegmentNum;
  int max_other_objects_num = 33;
  int max_lc_num = 64;
  int max_lb_num = 32;
  int max_crosswalk_num = 8;
  int max_dp_num = 80;
  double length_scale = 10.0;
  double width_scale = 3.0;
} kCutinSLConfig;  // NOLINT

struct CutinSLFeature {
  std::string agent_id;

  // Objects and agent features
  CutinSLObjectAbsoluteFeature agent_sl_feature;
  std::vector<CutinSLObjectFeatures> objs_sl_features;

  // Map features
  std::vector<CutinSLPolylineFeatures> lc_sl_features;
  std::vector<CutinSLPolylineFeatures> lb_sl_features;
  std::vector<CutinSLPolylineFeatures> cw_sl_features;

  // Map masks
  std::vector<float> lc_masks;
  std::vector<float> lb_masks;
  std::vector<float> cw_masks;

  // dp xy seg
  std::vector<float> dp_xy_seg;

  // Lane boudaries features (broken lines)
  std::vector<float> left_lane_boundary;
  std::vector<float> right_lane_boundary;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_CUTIN_FEATURE_H_
