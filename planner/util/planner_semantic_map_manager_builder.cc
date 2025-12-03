#include "onboard/planner/util/planner_semantic_map_manager_builder.h"

#include <stdint.h>

#include <algorithm>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"

#include "onboard/async/async_util.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/online_map_utils.h"
#include "onboard/maps/v2/semantic_map_definition.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner {

absl::StatusOr<std::shared_ptr<PlannerSemanticMapManager>> BuildOnlineMapPsmm(
    const mapping::OnlineSemanticMapProto& proto, ThreadPool* thread_pool) {
  FUNC_QTRACE();

  auto [semantic_map, meta] = mapping::ToSemanticMapProtos(proto);
  auto lt = proto.localization_transform();
  lt.set_level_id(0);
  CoordinateConverter cc = CoordinateConverter::FromLocalizationTransform(lt);

  auto smm = mapping::v2::SemanticMapManager::MakeSharedWithSpecifiedOption(
      mapping::v2::kInvalidUpdateId, {}, cc, {}, std::move(semantic_map),
      std::move(meta));
  QCHECK_NOTNULL(smm);
  smm->SetDataSource(OnlineMapProto::QCRAFT_VISIONMAP);

  const auto smmsi =
      mapping::v2::SemanticMapMultilevelSpatialIndex::MakeShared(smm);

  auto psmm = std::make_shared<PlannerSemanticMapManager>(smmsi);
  psmm->UpdateCoordinateConverter(cc);
  const auto build_status = psmm->BuildSemanticMapInfo(thread_pool);
  if (UNLIKELY(!build_status.ok())) {
    return absl::InternalError(absl::StrCat(
        "Build elements of PSMM failed. loc: ", QCRAFT_LOC.ToString()));
  }

  return psmm;
}

Future<std::shared_ptr<PlannerSemanticMapManager>>
AsyncLoadPlannerSemanticMapManager(
    const std::shared_ptr<mapping::v2::SemanticMapMultilevelSpatialIndex>&
        smmsi,
    const CoordinateConverter& cc, ThreadPool* thread_pool) {
  SCOPED_QTRACE(
      "PlannerSemanticMapManager::AsyncLoadPlannerSemanticMapManager");
  const int64_t call_time = absl::ToUnixMicros(Clock::Now());
  return ScheduleFuture(
      thread_pool,
      [smmsi, cc, thread_pool,
       call_time]() -> std::shared_ptr<PlannerSemanticMapManager> {
        if (smmsi == nullptr) return nullptr;
        auto psmm = std::make_shared<PlannerSemanticMapManager>(smmsi);
        psmm->UpdateCoordinateConverter(cc);
        const auto build_status = psmm->BuildSemanticMapInfo(thread_pool);
        if (UNLIKELY(!build_status.ok())) {
          QLOG(ERROR) << "Async build planner semantic map manager failed, "
                      << build_status.message();
          return nullptr;
        }
        QLOG(INFO)
            << "Async build planner semantic map manager success, update id: "
            << smmsi->semantic_manager()->update_id() << "\n"
            << psmm->DebugString();
        QLOG(INFO) << "Async build Psmm start time(us): " << call_time
                   << ", duration: "
                   << static_cast<double>(
                          (absl::ToUnixMicros(Clock::Now()) - call_time)) *
                          1e-3
                   << " ms.";
        return psmm;
      });
}

std::shared_ptr<mapping::v2::SemanticMapMultilevelSpatialIndex>
UpdateHdSemanticMapManagerAlongRoute(
    const Vec2d& global_pos, const RouteSectionSequenceProto& section_seq,
    const std::optional<PlannerState::HdMapState>& hd_map_state,
    const ::google::protobuf::RepeatedPtrField<OverlappingSection>*
        overlap_sections,
    mapping::v2::SemanticMapMultilevelSpatialIndexListenerAsnyc*
        hd_map_listener) {
  SCOPED_QTRACE("UpdateHdSemanticMapManagerAlongRoute");
  hd_map_listener->UpdateLonLat(global_pos.x(), global_pos.y());

  auto mpp_sections = std::make_shared<MppSectionsProto>();
  if (!section_seq.section_id().empty()) {
    auto* mutable_section_ids = mpp_sections->mutable_section_ids();
    const auto& route_section_ids = section_seq.section_id();
    mutable_section_ids->Reserve(route_section_ids.size());
    *mutable_section_ids = {route_section_ids.begin(), route_section_ids.end()};
    mpp_sections->set_ego_section_id(route_section_ids[0]);
    mpp_sections->set_ego_section_fraction(section_seq.start_fraction());
  }

  const bool precomputed_overlap = overlap_sections != nullptr;
  mpp_sections->set_overlapping_sections_precomputed(precomputed_overlap);
  if (precomputed_overlap) {
    *mpp_sections->mutable_overlapping_sections() = *overlap_sections;
  }

  // NOTE(zuowei): After switching to AOM, it is no longer necessary to call
  // ForceUpdate.
  if (!precomputed_overlap && !section_seq.section_id().empty() &&
      hd_map_state.has_value() && hd_map_state->load_distance > 0.0 &&
      hd_map_state->load_distance < kOnlineHdMapLoadMinDist &&
      !hd_map_state->has_destination) {
    hd_map_listener->ForceUpdateMppSections(mpp_sections);
  } else {
    hd_map_listener->UpdateMppSections(mpp_sections);
  }

  return hd_map_listener->Get();
}

}  // namespace qcraft::planner
