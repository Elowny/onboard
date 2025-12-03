#include "onboard/planner/decision/lc_end_of_current_lane_constraint.h"

#include <memory>
#include <optional>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
TEST(LaneChangeEndOfCurrentLaneConstraintTest, BasicTest) {
  // Load planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const auto lane_path_before_lc = mapping::LanePath(
      smm, /*lane_ids=*/{mapping::ElementId(2448), mapping::ElementId(1)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  const auto target_lane_path = mapping::LanePath(
      smm, /*lane_ids=*/
      {mapping::ElementId(3), mapping::ElementId(951), mapping::ElementId(43)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  const auto drive_passage = *BuildDrivePassageFromLanePath(
      psmm, target_lane_path, /*step_s=*/1.0,
      /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
      /*required_planning_horizon=*/0.0,
      /*required_backward_len=*/0.0,
      /*override_speed_limit_mps=*/std::nullopt);

  ASSIGN_OR_DIE(const auto speed_region,
                BuildLcEndOfCurrentLaneConstraints(
                    drive_passage, lane_path_before_lc, /*ego_v=*/15.0));

  EXPECT_NEAR(speed_region.start_s(), 50.193, 1e-3);
  EXPECT_NEAR(speed_region.end_s(), 70.193, 1e-3);
  EXPECT_NEAR(speed_region.max_speed(), 8.626, 1e-3);
}
}  // namespace
}  // namespace qcraft::planner
