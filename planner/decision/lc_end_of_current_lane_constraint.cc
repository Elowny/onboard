#include "onboard/planner/decision/lc_end_of_current_lane_constraint.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

absl::StatusOr<ConstraintProto::SpeedRegionProto>
BuildLcEndOfCurrentLaneConstraints(const DrivePassage& dp,
                                   const mapping::LanePath& lane_path_before_lc,
                                   double ego_v) {
  const double end_s = std::min(
      dp.end_s(), lane_path_before_lc.length() + dp.lane_path_start_s());
  // Set positive start_s so the ego vehicle really decelerates.
  constexpr double kMinEntryDistance = 10.0;  // m.
  const double start_s =
      std::min(end_s, std::max(kMinEntryDistance, end_s - kMinLcLaneLength));

  ConstraintProto::SpeedRegionProto proto;
  proto.set_start_s(start_s);
  proto.set_end_s(end_s);

  ASSIGN_OR_RETURN(const auto start_pt, dp.QueryPointXYAtS(proto.start_s()));
  start_pt.ToProto(proto.mutable_start_point());

  ASSIGN_OR_RETURN(const auto end_pt, dp.QueryPointXYAtS(proto.end_s()));
  end_pt.ToProto(proto.mutable_end_point());

  constexpr double kComfortableDeceleration = 1.5;
  const double entry_speed = std::max(
      kMinLCSpeed,
      std::sqrt(std::max(Sqr(ego_v) - 2.0 * kComfortableDeceleration * start_s,
                         0.0)));
  proto.set_min_speed(kMinLCSpeed);
  proto.set_max_speed(entry_speed);

  const std::string id = "lc_end_of_current_lane";
  proto.set_id(id);
  proto.mutable_source()->mutable_lc_end_of_current_lane()->set_id(id);

  return proto;
}

}  // namespace qcraft::planner
