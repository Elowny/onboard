#include "onboard/planner/scheduler/straighten_reference_line.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft::planner {
namespace {
TEST(StraightenReferenceLineTest,
     BuildStraightenedRampResultMapFromRouteSections) {
  const auto& psmm = CreateDojoTestPSMM();
  const RouteSections route_sections(
      /*start_fraction=*/0.0, /*end_fraction=*/1.0, {mapping::SectionId(12401)},
      mapping::LanePoint(mapping::ElementId(2448), 0.1));
  SmoothedReferenceLineResultMap prev_result;
  const auto results_or = BuildStraightenedRampResultMapFromRouteSections(
      psmm, route_sections, /*half_av_width=*/1.5, prev_result);
  EXPECT_OK(results_or);
  // TODO(zuowei): Supply unittest when dojo or navinfo map supports.
}
}  // namespace
}  // namespace qcraft::planner
