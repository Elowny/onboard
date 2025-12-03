#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
#include <map>
#include <string>
#include <vector>

#include "absl/status/status.h"

#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_defs.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/proto/charts.pb.h"

namespace qcraft {
namespace planner {
namespace optimizer {

std::optional<std::vector<TrajectoryPoint>>
AdaptTrajectoryToGivenPlanStartPoint(int trajectory_steps, const Mfob& problem,
                                     const DdpOptimizerParamsProto& params,
                                     double max_adaption_cost,
                                     const TrajectoryPoint& plan_start_point,
                                     std::vector<TrajectoryPoint> trajectory);

void AddTrajCharts(
    const std::string& base_name, const std::vector<TrajectoryPlotInfo>& trajs,
    google::protobuf::RepeatedPtrField<vis::vantage::ChartDataProto>*
        charts_data);

void AddCompareTrajCanvas(const std::string& base_name,
                          const std::vector<TrajectoryPoint>& first_traj,
                          const std::string& first_name,
                          const std::vector<TrajectoryPoint>& second_traj,
                          const std::string& second_name);

absl::Status CompareWithIpopt(
    const std::string& base_name, const std::string& canvas_base_name,
    const std::vector<TrajectoryPoint>& init_traj,
    const std::vector<TrajectoryPoint>& smooth_init_traj,
    const std::vector<TrajectoryPoint>& result_traj,
    bool enable_comparison_debug_info_output,
    const DdpOptimizerDebugProto& ddp_debug_proto,
    const TrajectoryPlotInfo& ddp_result_traj, const optimizer::Mfob* problem,
    vis::vantage::ChartDataBundleProto* charts_data);

absl::Status ValidateTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points,
    const TrajectoryOptimizerValidationParamsProto&
        trajectory_optimizer_validation_params,
    const TrajectoryOptimizerDebugProto& optimizer_debug);

// This function checks two trajectory have same decision using the following
// method: https://qcraft.feishu.cn/docs/doccnhyPYXRBDPkL8PbgIE4CCVf
bool HasSameDecisionOverSpacetimeObject(
    const std::vector<TrajectoryPoint>& traj_1,
    const std::vector<TrajectoryPoint>& traj_2,
    const std::vector<SpacetimeObjectState>& space_time_object_states);

absl::StatusOr<std::string_view> ExtractStationaryNudgeObjectId(
    const std::vector<TrajectoryPoint>& result_points,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
