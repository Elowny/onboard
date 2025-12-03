#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_QC_NET_FEATURE_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_QC_NET_FEATURE_H_

#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {

[[maybe_unused]] static constexpr struct QcNetConfig {
  // Temporary defs!
  int max_pred_num = 64;
  double future_time_step = 0.1;
  int future_num = 80;
  int coord_num = 2;
  int out_coord_num = 5;
  int trajectory_num = 3;
  int history_num = 10;
  double history_step_len = 0.1;
  int max_other_objs_num = 31;
  int obj_type_dim = 16;
  int max_lc_num = 160;
  int max_lb_num = 64;
  int max_cw_num = 16;
  int max_lane_seg_num = 4;
  int lane_type_dim = 16;
  int lane_seg_num = kFeatureV2MapSegmentNum;
  int rel_feature_dim = 4;
} kQcNetConfig;

struct QcNetLocalCoord {
  Vec2d pos;
  double heading;
  double rot_cos;
  double rot_sin;
};

struct QcNetObjectFeature {
  std::string id;
  QcNetLocalCoord coord;
  float category;                    // To one hot later.
  std::vector<float> velocity;       // Velocity vector relative to local coord.
  std::vector<float> motion_vector;  // ActNet pos_diff rotate to local coord.
  std::vector<float> rel_temporal_pos;  // ActNet pos relative to local coord.
};

struct QcNetPolylineFeature {
  mapping::ElementId id = mapping::kInvalidElementId;
  // TODO(changqing): check to to see whether it's necessary.
  int index;
  QcNetLocalCoord coord;          // first seg start point + first seg heading.
  std::vector<float> points_pos;  // Seg end point relative to local coord.
  std::vector<float> type;        // To one hot later.
};

// Use const pointer or const span. All features in cache. This structure is not
// necessary to be constructed in j5 inferencer.
struct QcNetFeature {
  std::string agent_id;
  const QcNetObjectFeature* agent_feature;
  absl::Span<const QcNetObjectFeature* const> context_obj_features;
  std::vector<float> context_obj_masks;

  absl::Span<const QcNetPolylineFeature* const> lc_features;
  absl::Span<const QcNetPolylineFeature* const> lb_features;
  absl::Span<const QcNetPolylineFeature* const> cw_features;
  std::vector<float> lc_masks;
  std::vector<float> lb_masks;
  std::vector<float> cw_masks;

  // Need to compute based on chosen agent, objs, polylines. Temporal relative
  // features are in object features.
  // TODO(changqing): Temporary container to store data. Fill Tensor directly
  // later.
  std::vector<float> rel_agent_map;
  std::vector<float> rel_map_map;
  std::vector<float> rel_agent_obj;
};

// Related to ActNetQcJ5.
// Qc local coord features.
struct ActNetQcObjectFeature {
  AgentCoordTransform coord_trans;
  std::vector<float> pos;
  std::vector<float> pos_diff;
  std::vector<float> heading;
  std::vector<float> speed;
  float type;
  std::array<float, 2> lw;
};

struct ActNetQcPolylineLocalFeature {
  AgentCoordTransform coord_trans;
  std::vector<float> seg;
  std::vector<float> seg_len;
  std::vector<float> unit_tangent;
  std::vector<float> unit_seg_vec;
  std::vector<float> speed_limit_kph;
  std::vector<float> type;
  std::vector<float> light;
};

struct ActNetQcPolylineFeature {
  const ActNetQcPolylineLocalFeature* abs_feat = nullptr;
  std::vector<float> agent_rel_feat;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_QC_NET_FEATURE_H_
