#include "onboard/planner/decision/toll_decider.h"

#include <optional>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
TEST(TollDeciderTest, BaseTest) {
  const auto& psmm = CreateDojoTestPSMM();
  const mapping::ElementId endpoint_toll_lane_id(7471);
  // Mock endpoint toll lane proto.
  {
    const mapping::LaneProto* lane_proto =
        psmm.FindLaneByIdOrNull(endpoint_toll_lane_id);
    auto* mutable_lane_proto = const_cast<mapping::LaneProto*>(lane_proto);
    mutable_lane_proto->set_endpoint_toll(true);
  }

  const auto current_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {endpoint_toll_lane_id, mapping::ElementId(7757)},
                        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, current_lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  ASSIGN_OR_DIE(const auto speed_regions,
                BuildTollConstraints(psmm, drive_passage, current_lane_path,
                                     /*s_offset*/ 0.0));
  EXPECT_EQ(speed_regions.size(), 1);
  const auto& speed_region = speed_regions.front();
  EXPECT_NEAR(speed_region.start_s(), 78.9, 1e-1);
  EXPECT_NEAR(speed_region.end_s(), 83.9, 1e-1);
}

}  // namespace
}  // namespace qcraft::planner
