#ifndef ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_MANAGER_BUILDER_H_
#define ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_MANAGER_BUILDER_H_

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <memory>
#include <optional>

#include "absl/status/statusor.h"

#include "onboard/async/future.h"
#include "onboard/async/thread_pool.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_listener.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_state.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner {

absl::StatusOr<std::shared_ptr<PlannerSemanticMapManager>> BuildOnlineMapPsmm(
    const mapping::OnlineSemanticMapProto& proto,
    ThreadPool* thread_pool = nullptr);

Future<std::shared_ptr<PlannerSemanticMapManager>>
AsyncLoadPlannerSemanticMapManager(
    const std::shared_ptr<mapping::v2::SemanticMapMultilevelSpatialIndex>&
        smmsi,
    const CoordinateConverter& cc, ThreadPool* thread_pool);

std::shared_ptr<mapping::v2::SemanticMapMultilevelSpatialIndex>
UpdateHdSemanticMapManagerAlongRoute(
    const Vec2d& global_pos, const RouteSectionSequenceProto& section_seq,
    const std::optional<PlannerState::HdMapState>& hd_map_state,
    const ::google::protobuf::RepeatedPtrField<OverlappingSection>*
        overlap_sections,
    mapping::v2::SemanticMapMultilevelSpatialIndexListenerAsnyc*
        hd_map_listener);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_MANAGER_BUILDER_H_
