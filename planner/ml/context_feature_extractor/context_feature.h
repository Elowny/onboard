#ifndef ONBOARD_PLANNER_ML_CONTEXT_FEATURE_H_
#define ONBOARD_PLANNER_ML_CONTEXT_FEATURE_H_

#include <string>
#include <vector>

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/ml/planner_ml_defs.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace planner {
namespace ml {

inline constexpr struct ActNetContextConfig {
  double history_time_step = kHistoryStepLen;  // s.
  double history_step_num = kHistoryStepNum;   // s.
  int coord_num = 2;
  int max_lane_seg_num = kMapSegmentNum;
  int max_lane_sample_len = kMaxMapSampleLen;
  int max_objects_num = 32;
  int max_lc_num = 160;
  int max_lb_num = 80;
  int max_crosswalk_num = 16;
  double scan_box_front = 150.0;
  double scan_box_back = 30.0;
  double scan_box_half_width = 60.0;
} kActNetContextConfig;

struct ObjectInferredFeature {
  std::string object_id;
  bool is_static;
  bool has_longterm_observation;
  float average_speed;
  float observation_duration;
};

struct InferredFeature {
  std::vector<ObjectInferredFeature> inferred_object_feature;
};

struct ContextFeature {
  double current_ts;  // s.
  Vec2d ref_position;
  float rot_rad;
  prediction::ActNetFeature act_net_feature;
  InferredFeature inferred_feature;
};

}  // namespace ml
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ML_CONTEXT_FEATURE_H_
