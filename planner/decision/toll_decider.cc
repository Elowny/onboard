#include "onboard/planner/decision/toll_decider.h"

#include <algorithm>
#include <ostream>
#include <string>
#include <utility>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {
namespace {
// Used to compare double values.
constexpr double kEpsilon = 1e-5;

// The deceleration distance of approaching toll
constexpr double kTollApproachingDecelerationDistance = 5.0;  // m

// The speed of approaching toll
constexpr double kTollApproachingSpeed = 2.0;  // m/s

}  // namespace
absl::StatusOr<std::vector<ConstraintProto::SpeedRegionProto>>
BuildTollConstraints(const PlannerSemanticMapManager& psmm,
                     const DrivePassage& passage,
                     const mapping::LanePath& lane_path_from_start,
                     double s_offset) {
  std::vector<ConstraintProto::SpeedRegionProto> toll_speed_regions;

  for (const auto& lane_seg : lane_path_from_start) {
    const auto* lane_ptr = psmm.FindLaneByIdOrNull(lane_seg.lane_id);
    if (lane_ptr == nullptr || !lane_ptr->has_endpoint_toll() ||
        !lane_ptr->endpoint_toll()) {
      continue;
    }
    // toll at the end of lane
    const double toll_s =
        lane_seg.end_s + s_offset + passage.lane_path_start_s();

    // ignore toll behind AV
    if (toll_s < kEpsilon) continue;

    const double start_s = toll_s - kTollApproachingDecelerationDistance;
    const double end_s = toll_s;
    const auto start_point = passage.QueryPointXYAtS(start_s);
    const auto end_point = passage.QueryPointXYAtS(end_s);

    // ignore toll which exceeds the length of drive passage
    if (!start_point.ok() || !end_point.ok()) continue;
    ConstraintProto::SpeedRegionProto speed_region;

    start_point->ToProto(speed_region.mutable_start_point());
    end_point->ToProto(speed_region.mutable_end_point());
    speed_region.set_start_s(start_s);
    speed_region.set_end_s(end_s);
    speed_region.set_max_speed(kTollApproachingSpeed);
    speed_region.set_min_speed(0.0);
    speed_region.set_id(absl::StrCat("toll in lane:", lane_seg.lane_id));
    speed_region.mutable_source()->mutable_toll()->set_id(
        lane_seg.lane_id.value());
    VLOG(2) << " + + + Generate speed region for toll,at:\t"
            << speed_region.start_s() << " | " << speed_region.end_s() << " | "
            << speed_region.max_speed();
    toll_speed_regions.emplace_back(std::move(speed_region));
  }

  return toll_speed_regions;
}
}  // namespace planner

}  // namespace qcraft
