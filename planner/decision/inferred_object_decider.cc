#include "onboard/planner/decision/inferred_object_decider.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"

#include "common/proto/lane_point.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/maps/lane_speed_limit.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/proto/piecewise_linear_function.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner {

namespace {

constexpr double kDefaultBraking = -1.0;  // m/s^2
constexpr double kMinSpeedFraction = 0.75;

struct InferredObjectInfo {
  mapping::ElementId lane_id;
  double lane_frac, dist_to_interaction;
  std::string object_id;
};

PiecewiseLinearFunctionDoubleProto CreateSpeedProfile(double init_v, double a,
                                                      double min_v) {
  constexpr double kTimeInterval = 1.0;  // seconds.
  const int n = static_cast<int>(kTrajectoryTimeHorizon / kTimeInterval) + 1;

  PiecewiseLinearFunctionDoubleProto speed_profile;
  speed_profile.mutable_x()->Reserve(n);
  speed_profile.mutable_y()->Reserve(n);
  for (double t = 0.0; t < kTrajectoryTimeHorizon; t += kTimeInterval) {
    speed_profile.add_x(t);
    speed_profile.add_y(std::max(init_v + a * t, min_v));
  }

  return speed_profile;
}

}  // namespace

absl::StatusOr<ConstraintProto::SpeedProfileProto>
BuildInferredObjectConstraint(const PlannerSemanticMapManager& psmm,
                              const SceneOutputProto& scene_reasoning,
                              const mapping::LanePath& lane_path_from_start,
                              double ego_init_v) {
  std::vector<InferredObjectInfo> object_infos;
  for (const auto& inferred_object : scene_reasoning.inferred_objects()) {
    if (inferred_object.infer_source().type_case() !=
        InferredObjectProto::InferSource::TypeCase::kInteractionLane) {
      continue;
    }
    const auto& interaction_info =
        inferred_object.infer_source().interaction_lane();
    object_infos.push_back(InferredObjectInfo{
        .lane_id =
            mapping::ElementId(interaction_info.interaction_point().lane_id()),
        .lane_frac = interaction_info.interaction_point().fraction(),
        .dist_to_interaction = interaction_info.dist_to_interaction(),
        .object_id = inferred_object.object_info().id()});
  }

  for (const auto& seg : lane_path_from_start) {
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, seg.lane_id);
    const auto& interactions = lane_info.proto->interactions();
    for (const auto& interaction : interactions) {
      const auto other_lane_id =
          mapping::ElementId(interaction.other_lane_id());
      const double other_lane_frac = interaction.other_lane_fraction();
      SMM_ASSIGN_LANE_OR_CONTINUE(other_lane_info, psmm, other_lane_id);

      for (const auto& object_info : object_infos) {
        if (object_info.lane_id != other_lane_id ||
            (other_lane_frac - object_info.lane_frac) *
                    other_lane_info.length() >
                object_info.dist_to_interaction) {
          continue;
        }

        // Use original speed limit.
        const double min_speed =
            kMinSpeedFraction *
            mapping::SpeedLimitAtFractionOrDefault(lane_info.speed_limits,
                                                   object_info.lane_frac);
        auto speed_profile =
            CreateSpeedProfile(ego_init_v, kDefaultBraking, min_speed);
        ConstraintProto::SpeedProfileProto speed_profile_constraint;
        *speed_profile_constraint.mutable_vt_upper_constraint() =
            std::move(speed_profile);
        speed_profile_constraint.mutable_source()
            ->mutable_occluded_object()
            ->set_id(object_info.object_id);

        QEVENT_EVERY_N_SECONDS("zixuan", "prebrake_for_inferred_object",
                               /*seconds=*/5.0, [](QEvent* /*qevent*/) {});

        return speed_profile_constraint;
      }
    }
  }

  return absl::NotFoundError("no valid inferred object");
}

}  // namespace qcraft::planner
