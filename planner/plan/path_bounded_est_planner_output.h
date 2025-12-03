
#ifndef ONBOARD_PLANNER_PLAN_PATH_BOUNDED_EST_PLANNER_OUTPUT_H_
#define ONBOARD_PLANNER_PLAN_PATH_BOUNDED_EST_PLANNER_OUTPUT_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"

#include "onboard/planner/assist/plc_internal_result.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/selector/proto/selector_debug.pb.h"
#include "onboard/planner/selector/selector_output.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::planner {

struct PathBoundedEstPlannerOutput {
  std::vector<PlannerStatus> est_status_list;
  std::vector<EstPlannerOutput> est_planner_output_list;
  std::vector<EstPlannerDebug> est_planner_debug_list;
  std::vector<vis::vantage::ChartDataBundleProto> chart_data_list;
  std::vector<PathPoint> st_path_points_global_including_past;

  EstPlannerDebug async_high_freq_debug;
  std::optional<RouteTargetInfo> route_target_info;  // For async planner.
  bool scheduled_async_low_freq = false;

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  std::optional<SelectorOutput> selector_output = std::nullopt;

  std::optional<PlcInternalResult> plc_result = std::nullopt;  // For teleop lc.
  QALCState alc_state;

  // Optimizer Auto Tuning
  AutoTuningDataProto auto_tuning_data;

  ObjectsPredictionProto speed_considered_objects_prediction;

  int path_start_relative_index = 0;

  std::shared_ptr<const PlannerSemanticMapManager> low_freq_psmm = nullptr;
  std::shared_ptr<const DrivingMapTopo> driving_map_topo = nullptr;

  // Alcc only.
  LaneChangeDirection lc_direction;
  mapping::LanePath origin_lane_path;
  mapping::LanePath target_lane_path;
  int64_t online_map_id = kInvalidOnlineMapId;

  // Both cruise and alcc task will assign a value to
  // 'st_path_points_including_past'. Especially when task downgrade from cruise
  // to alcc, 'st_path_points_including_past' will exhibit continuity.
  std::vector<PathPoint> st_path_points_including_past;

  // Hmi
  std::optional<NudgeOjbectInfo> nudge_object_info;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_PATH_BOUNDED_EST_PLANNER_OUTPUT_H_
