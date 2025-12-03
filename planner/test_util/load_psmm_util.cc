#include "onboard/planner/test_util/load_psmm_util.h"

#include <mutex>
#include <tuple>
#include <utility>

#include "onboard/maps/map_selector.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/math/coordinate_converter.h"

namespace qcraft::planner {

std::shared_ptr<PlannerSemanticMapManager> CreateDojoTestPSMMSharedPtr() {
  static std::once_flag psmm_v2_load_dojo_for_test;
  static std::shared_ptr<PlannerSemanticMapManager> psmm = nullptr;
  std::call_once(psmm_v2_load_dojo_for_test, []() {
    SetMap("dojo");
    auto loader = mapping::v2::SemanticMapLoader::MakeShared();
    const auto smm_v2 = loader->PreloadWholeMap().Get();
    auto smmsi =
        mapping::v2::SemanticMapMultilevelSpatialIndex::MakeShared(smm_v2);

    const CoordinateConverter cc = CoordinateConverter::FromLocale();

    psmm = std::make_shared<PlannerSemanticMapManager>(std::move(smmsi));

    psmm->UpdateCoordinateConverter(cc);
    std::ignore = psmm->BuildSemanticMapInfo(/*ThreadPool=*/nullptr);
  });
  return psmm;
}

// TODO(zuowei): Figure out why returning `*CreateDojoTestPSMMSharedPtr()`
// directly is problematic.
const PlannerSemanticMapManager& CreateDojoTestPSMM() {
  static std::once_flag psmm_v2_load_dojo_for_test;
  static PlannerSemanticMapManager* psmm = nullptr;
  std::call_once(psmm_v2_load_dojo_for_test, []() {
    SetMap("dojo");
    auto loader = mapping::v2::SemanticMapLoader::MakeShared();
    const auto smm_v2 = loader->PreloadWholeMap().Get();
    const auto smmsi =
        mapping::v2::SemanticMapMultilevelSpatialIndex::MakeShared(smm_v2);

    const CoordinateConverter cc = CoordinateConverter::FromLocale();

    psmm = new PlannerSemanticMapManager(smmsi);

    psmm->UpdateCoordinateConverter(cc);
    std::ignore = psmm->BuildSemanticMapInfo(/*ThreadPool=*/nullptr);
  });
  return *psmm;
}

}  // namespace qcraft::planner
