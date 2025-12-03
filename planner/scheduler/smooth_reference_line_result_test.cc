#include "onboard/planner/scheduler/smooth_reference_line_result.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/scheduler/smooth_reference_line_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft::planner {
namespace {

TEST(SmoothedReferenceLineResultMap, SmoothResultTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  SmoothedReferenceLineResultMap results;
  {
    const mapping::ElementId id(7814);
    const std::vector<mapping::ElementId> lane_ids = {id};
    const auto smoothed_line_or =
        SmoothLanePathByLaneIds(psmm, lane_ids, /*half_av_width=*/0.0);

    EXPECT_OK(smoothed_line_or);
    EXPECT_TRUE(!results.Contains(lane_ids));
    results.AddResult(lane_ids, *smoothed_line_or);
    EXPECT_TRUE(results.Contains(lane_ids));
    results.DeleteResult(lane_ids);
    EXPECT_TRUE(!results.Contains(lane_ids));
  }

  results.Clear();
  EXPECT_TRUE(results.smoothed_result_map().empty());
  {
    const mapping::ElementId id(7806);
    const std::vector<mapping::ElementId> lane_ids = {id};
    const auto smoothed_line_or =
        SmoothLanePathByLaneIds(psmm, lane_ids, /*half_av_width=*/0.0);

    EXPECT_OK(smoothed_line_or);
    EXPECT_TRUE(!results.Contains(lane_ids));
    results.AddResult(lane_ids, *smoothed_line_or);
    EXPECT_TRUE(results.Contains(lane_ids));
    results.DeleteResult(lane_ids);
    EXPECT_TRUE(!results.Contains(lane_ids));
  }
}

}  // namespace
}  // namespace qcraft::planner
