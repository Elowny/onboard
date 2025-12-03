#ifndef ONBOARD_PLANNER_INITIALIZER_INITIALIZER_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_INITIALIZER_UTIL_H_

#include <map>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"

#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

void SendSingleTrajectoryToCanvas(
    const MotionSearchOutput::MultiTrajCandidate& traj_info, int idx,
    int planner_id);

void SendGeometryGraphToCanvas(const GeometryGraph* graph,
                               const std::string& channel);

void SendRefSpeedTableToCanvas(const RefSpeedTable& ref_speed_table,
                               const DrivePassage& drive_passage);

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>>
GenerateConstLateralAccelConstSpeedTraj(
    const DrivePassage& drive_passage, double ego_front_to_ra, double target_l,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    absl::Span<const ConstraintProto::StopLineProto> stop_line,
    const ApolloTrajectoryPointProto& plan_start_point, int traj_steps);

void ParseMotionSearchOutputToMotionSearchDebugProto(
    const MotionSearchOutput& search_output, MotionSearchDebugProto* proto);

void ParseMotionSearchOutputToMultiTrajDebugProto(
    const MotionSearchOutput& search_output, MultiTrajDebugProto* proto);

void ParseMotionSearchOutputToInitializerResult(
    const MotionSearchOutput& search_output, InitializerDebugProto* proto);

void ExportMoionSpeedProfileToChart(const MotionSearchOutput& search_output,
                                    vis::vantage::ChartDataProto* chart);

InitializerOutput MakeAebInitializerOutput(
    int traj_steps, ApolloTrajectoryPointProto plan_start_point,
    InitializerStateProto new_state, absl::string_view message,
    InitializerDebugProto* debug_proto);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_INITIALIZER_UTIL_H_
