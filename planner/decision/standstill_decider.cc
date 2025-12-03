#include "onboard/planner/decision/standstill_decider.h"

#include <algorithm>
#include <string>
#include <utility>

#include "absl/strings/str_cat.h"

#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/vec.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {
// Used to compare double values.
constexpr double kEpsilon = 1e-5;

// if there is a stop line in front of the AV, the space between is less than
// default standstill distance, when the AV stops, the AV will not speed up
// again, until stop line disappears.
constexpr double kDefaultStandstillDistance = 3.0;  // m

// if the speed of AV is less than this value,  it is considered to have
// stopped.
constexpr double kStopSpeedThreshold = 0.2;  // m/s

// Distance condition, check whether generating standstill stop line.
constexpr double kDefaultStandstillThreshold = 5.0;  // m

absl::StatusOr<ConstraintProto::StopLineProto> GenerateStandstillConstraints(
    const double& av_front_edge_s, const std::string& reason,
    const DrivePassage& passage) {
  ASSIGN_OR_RETURN(const auto curbs,
                   passage.QueryCurbPointAtS(av_front_edge_s));

  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(av_front_edge_s);
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(absl::StrCat("standstill"));
  stop_line.mutable_source()->mutable_standstill()->set_reason(reason);

  return stop_line;
}

// Adding standstill stop line for kTrafficLight/kCrosswalk/kPullOver three
// kinds of stop line. If new TypeCase added, just return false.
bool IsConsideredStopLineType(const SourceProto::TypeCase& type) {
  switch (type) {
    case SourceProto::TypeCase::kTrafficLight:
    case SourceProto::TypeCase::kCrosswalk:
    case SourceProto::TypeCase::kPullOver:
    case SourceProto::TypeCase::kBrakeToStop:
      return true;
    case SourceProto::TypeCase::kCloseObject:
    case SourceProto::TypeCase::kSpeedBump:
    case SourceProto::TypeCase::kIntersection:
    case SourceProto::TypeCase::kLcEndOfCurrentLane:
    case SourceProto::TypeCase::kPedestrianObject:
    case SourceProto::TypeCase::kToll:
    case SourceProto::TypeCase::kNoBlock:
    case SourceProto::TypeCase::kEndOfPathBoundary:
    case SourceProto::TypeCase::kEndOfCurrentLanePath:
    case SourceProto::TypeCase::kRouteDestination:
    case SourceProto::TypeCase::kParkingBrakeRelease:
    case SourceProto::TypeCase::kBlockingStaticObject:
    case SourceProto::TypeCase::kStandby:
    case SourceProto::TypeCase::kStopSign:
    case SourceProto::TypeCase::kStandstill:
    case SourceProto::TypeCase::kEndOfLocalPath:
    case SourceProto::TypeCase::kBeyondLengthAlongRoute:
    case SourceProto::TypeCase::kSolidLineWithinBoundary:
    case SourceProto::TypeCase::kOccludedObject:
    case SourceProto::TypeCase::kDenseTrafficFlow:
    case SourceProto::TypeCase::kStopPolyline:
    case SourceProto::TypeCase::TYPE_NOT_SET:
      return false;
  }
}

}  // namespace
absl::StatusOr<std::vector<ConstraintProto::StopLineProto>>
BuildStandstillConstraints(
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage,
    absl::Span<const ConstraintProto::StopLineProto> stop_lines) {
  std::vector<ConstraintProto::StopLineProto> ss_stop_lines;

  if (stop_lines.empty()) return ss_stop_lines;

  const double plan_start_v = plan_start_point.v();
  if (plan_start_v > kStopSpeedThreshold) return ss_stop_lines;

  // calculate AV front edge sl coordinate on drive passage.
  const Vec2d plan_start_pos_xy(plan_start_point.path_point().x(),
                                plan_start_point.path_point().y());

  ASSIGN_OR_RETURN(const auto plan_start_pos_sl,
                   passage.QueryFrenetCoordinateAt(plan_start_pos_xy));

  const double av_front_edge_s =
      plan_start_pos_sl.s + vehicle_geometry_params.front_edge_to_center();

  for (const auto& stop_line : stop_lines) {
    const auto type = stop_line.source().type_case();
    if (!IsConsideredStopLineType(type)) continue;

    const double interval = stop_line.s() - av_front_edge_s;
    if (interval < kEpsilon || kDefaultStandstillThreshold < interval) continue;

    // generate stop line for standstill
    ASSIGN_OR_CONTINUE(auto ss_stop_line,
                       GenerateStandstillConstraints(
                           std::max(av_front_edge_s,
                                    stop_line.s() - kDefaultStandstillDistance),
                           stop_line.id(), passage));
    ss_stop_lines.emplace_back(std::move(ss_stop_line));
    break;
  }

  return ss_stop_lines;
}
}  // namespace planner
}  // namespace qcraft
