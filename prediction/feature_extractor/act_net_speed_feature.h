#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_SPEED_FEATURE_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_SPEED_FEATURE_H_

#include <array>
#include <string>
#include <vector>

#include "onboard/prediction/feature_extractor/act_net_feature.h"

namespace qcraft {
namespace prediction {

static constexpr struct ActNetSpeedConfig {
  int max_pred_num = 32;
  double future_time_step = 0.1;  // s.

  // Input coordinate dim.
  int coord_num = 2;

  // Output params.
  int future_num = 100;
  int mode_num = 2;  // two relation.

  // Feature extraction params. Path specific.
  int path_point_num_pre_intersection = 20;
  int path_point_num = 20;
  int path_coord_num = 6;  // x,y,s,s_offset,hcos,hsin.
  // TODO(changqing): Check if needed to accelerate analyzing trajectory
  // intersection point.
  double av_segment_min_distance = 3.0;

  // Copy from ActNet feature extractor.
  int max_lane_seg_num = kFeatureV2MapSegmentNum;
  int max_other_objects_num = 33;
  int max_lc_num = 160;
  int max_lb_num = 80;
  int max_crosswalk_num = 16;
  double scan_box_front = 150.0;
  double scan_box_back = 30.0;
  double scan_box_half_width = 60.0;
} kActNetSpeedConfig;

struct PathFeature {
  std::vector<float> path_xy;
  std::vector<float> path_s;
  std::vector<float> path_s_offset;
  std::vector<float> path_heading;
  std::vector<float> path_xy_preint;
  std::vector<float> path_s_preint;
  std::vector<float> path_s_offset_preint;
  std::vector<float> path_heading_preint;
  std::vector<float> point_valid;
};

struct ActNetSpeedFeature {
  std::string agent_id;
  int traj_index;  // Need to specify traj index to modify predicted trajectory
                   // struct.
  Vec2d ref_position;
  float rot_rad;
  ActNetObjectAbsoluteFeature agent_feature;
  std::vector<ActNetObjectFeature> context_obj_features;
  std::vector<float> context_obj_masks;
  // Speed model specific features.
  PathFeature agent_path;
  PathFeature av_path;
  // Agent complete pre-intersection path.
  // AV complete pre-intersection path.
  std::array<float, 2> path_valid_mask = {1.0, 1.0};
  std::string DebugString() const {
    return absl::StrFormat(
        "agent_id: %s, path_valid: %s, agent s_preint: %s, av s_preint: %s. "
        "agent_heading: %s, av_heading: %s. agent_xy: %s, av_xy: %s. av path s "
        "offset: %s, agent path s offset: %s.",
        agent_id, absl::StrJoin(path_valid_mask, ","),
        absl::StrJoin(agent_path.path_s_offset_preint, ","),
        absl::StrJoin(av_path.path_s_offset_preint, ","),
        absl::StrJoin(agent_path.path_heading_preint, ","),
        absl::StrJoin(av_path.path_heading_preint, ","),
        absl::StrJoin(agent_path.path_xy_preint, ","),
        absl::StrJoin(av_path.path_xy_preint, ","),
        absl::StrJoin(av_path.path_s_offset, ","),
        absl::StrJoin(agent_path.path_s_offset, ","));
  }
};

}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_SPEED_FEATURE_H_
