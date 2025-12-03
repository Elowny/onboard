#include "onboard/planner/scene/occluded_interaction_lane_reasoning.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <map>
#include <optional>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

constexpr double kDefaultVehicleLength = 4.5;      // m.
constexpr double kDefaultVehicleWidth = 2.0;       // m.
constexpr double kInteractionLaneLength = 100.0;   // m.
constexpr double kLaneSamplingStep = 1.0;          // m.
constexpr double kInferredObjectMinDist = 10.0;    // m.
constexpr double kMinOcclusionTimeToInfer = 2.0;   // s.
constexpr double kMaxOcclusionTime = 5.0;          // s.
constexpr double kRouteSectionCutoffDist = 150.0;  // m.

std::vector<mapping::LanePath> CollectInteractionLanesAlongRouteSections(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections) {
  std::map<mapping::ElementId, mapping::LanePath> lane_path_map;

  for (int i = 0; i < route_sections.size(); ++i) {
    const auto section_segment = route_sections.route_section_segment(i);
    SMM_ASSIGN_SECTION_OR_BREAK_ISSUE(section_info, psmm, section_segment.id);

    for (const auto lane_id : section_info.lane_ids) {
      SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm, lane_id);
      const auto& interactions = lane_info.proto->interactions();
      for (const auto& interaction : interactions) {
        const auto other_lane_id =
            mapping::ElementId(interaction.other_lane_id());
        const double other_start_frac = interaction.other_lane_fraction();
        // Ignore interactions that out of route section effective range.
        if (interaction.this_lane_fraction() < section_segment.start_fraction ||
            interaction.this_lane_fraction() > section_segment.end_fraction) {
          continue;
        }

        auto it = lane_path_map.find(other_lane_id);
        if (it == lane_path_map.end()) {
          // TODO(zixuan): Handle fork lanes for backward extension.
          mapping::LanePath lane_path(
              psmm.semantic_map_manager(),
              mapping::LanePoint(other_lane_id, other_start_frac));
          lane_path_map[other_lane_id] =
              BackwardExtendLanePath(psmm, lane_path, kInteractionLaneLength);
        } else {
          const auto& origin_lane_path = it->second;
          if (origin_lane_path.start_fraction() > other_start_frac) {
            mapping::LanePath new_lane_path(
                psmm.semantic_map_manager(), origin_lane_path.lane_ids(),
                other_start_frac, origin_lane_path.end_fraction());
            it->second = std::move(new_lane_path);
          }
        }
      }
    }
  }

  std::vector<mapping::LanePath> lane_path_vec;
  lane_path_vec.reserve(lane_path_map.size());
  for (auto& [_, lane_path] : lane_path_map) {
    lane_path_vec.emplace_back(std::move(lane_path));
  }

  return lane_path_vec;
}

std::vector<InferredObjectProto> ReasoningOccludedObjectsOnOneLane(
    const PlannerSemanticMapManager& psmm,
    const sensor_fov::SensorFov& sensor_fov,
    const absl::flat_hash_set<mapping::SectionId>& section_id_set,
    const mapping::LanePath& lane_path) {
  std::vector<InferredObjectProto> inferred_objects;
  int occluded_object_id = 1;
  std::optional<double> prev_sampled_s;

  for (double sample_s = lane_path.length(); sample_s >= 0.0;
       sample_s -= kLaneSamplingStep) {
    const auto sample_lane_point = lane_path.ArclengthToLanePoint(sample_s);
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, sample_lane_point.lane_id());
    if (section_id_set.contains(lane_info.section_id)) continue;

    const auto center = ComputeLanePointPos(psmm, sample_lane_point);
    const auto tangent = ComputeLanePointTangent(psmm, sample_lane_point);

    const Box2d box(center, tangent, kDefaultVehicleLength,
                    kDefaultVehicleWidth);
    ASSIGN_OR_CONTINUE(const auto occlusion_state, sensor_fov.IsOccluded(box));

    if (!occlusion_state.occluded ||
        occlusion_state.occlusion_time < kMinOcclusionTimeToInfer ||
        (prev_sampled_s.has_value() &&
         sample_s - *prev_sampled_s < kInferredObjectMinDist)) {
      continue;
    }

    const auto interaction_lane_point = lane_path.back();
    prev_sampled_s = sample_s;

    InferredObjectProto proto;
    proto.set_infer_type(InferredObjectProto::OCCLUDED);
    proto.set_confidence(std::clamp(
        occlusion_state.occlusion_time / kMaxOcclusionTime, 0.0, 1.0));

    auto* mutable_interaction_lane =
        proto.mutable_infer_source()->mutable_interaction_lane();
    interaction_lane_point.ToProto(
        mutable_interaction_lane->mutable_interaction_point());
    mutable_interaction_lane->set_dist_to_interaction(lane_path.length() -
                                                      sample_s);

    // Set objectproto.
    auto* object_proto = proto.mutable_object_info();
    object_proto->set_id(absl::StrCat("occ-lane-",
                                      interaction_lane_point.lane_id(), "-",
                                      occluded_object_id++));
    object_proto->set_type(ObjectType::OT_VEHICLE);
    Vec2dToProto(center, object_proto->mutable_pos());
    object_proto->set_yaw(tangent.FastAngle());
    Vec2dToProto(tangent * psmm.QueryLaneSpeedLimitByFraction(
                               sample_lane_point.lane_id(),
                               sample_lane_point.fraction()),
                 object_proto->mutable_vel());

    // Set countor and bounding box.
    for (const auto& pt : box.GetCornersCounterClockwise()) {
      Vec2dToProto(pt, object_proto->add_contour());
    }
    box.ToProto(object_proto->mutable_bounding_box());

    object_proto->set_parked(false);
    object_proto->set_offroad(false);
    object_proto->set_moving_state(ObjectProto::MS_MOVING);

    inferred_objects.emplace_back(std::move(proto));
  }

  return inferred_objects;
}

}  // namespace

// Reasoning occluded objects on the lanes having interaction with route
// section.
std::vector<InferredObjectProto> InferOccludedObjectsOnInteractionLanes(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const sensor_fov::SensorFov& sensor_fov) {
  FUNC_QTRACE();
  std::vector<InferredObjectProto> output_inferred_objects;

  ASSIGN_OR_RETURN(const auto clamped_route_sections,
                   ClampRouteSectionsBeforeArcLength(psmm, route_sections,
                                                     kRouteSectionCutoffDist),
                   output_inferred_objects);

  const auto lane_path_vec =
      CollectInteractionLanesAlongRouteSections(psmm, clamped_route_sections);

  absl::flat_hash_set<mapping::SectionId> section_id_set;
  for (const auto section_id : clamped_route_sections.section_ids()) {
    section_id_set.insert(section_id);
  }

  for (const auto& lane_path : lane_path_vec) {
    auto inferred_objects = ReasoningOccludedObjectsOnOneLane(
        psmm, sensor_fov, section_id_set, lane_path);
    for (auto& obj : inferred_objects) {
      output_inferred_objects.emplace_back(std::move(obj));
    }
  }

  return output_inferred_objects;
}

}  // namespace qcraft::planner
