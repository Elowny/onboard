#include "onboard/planner/decision/traffic_gap_v2/traffic_gap_finder_impl.h"

#include <float.h>

#include <algorithm>
#include <string>

#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {
namespace {

PlannerObject BuildPlannerObject(const std::string& obj_id,
                                 const Vec2d& obj_pos, double vel,
                                 double heading = 0.0) {
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id(obj_id)
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(obj_pos)
                            .set_length_width(4.2, 2.1)
                            .set_yaw(heading)
                            .set_velocity(vel)
                            .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .set_stationary(false)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.8)
      .set_straight_line(obj_pos,
                         obj_pos + Vec2d::FastUnitFromAngle(heading) * vel *
                                       prediction::kPredictionDuration,
                         /*init_v=*/vel, /*last_v=*/vel);
  return builder.Build();
}

TEST(TrafficGapFinder, ComputeGapAlignInfoWithGapBehind) {
  {
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/56, /*target_v=*/16.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/5.0, /*gap_behind=*/true, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 8.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 72.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 13.0, 1e-3);
  }
  {
    /*
    ============================  target_v
                          ------  final_v
                         /
    ====================/=======  align_v (not used)
    --------------------          ego_v
    */
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/11.5, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/1.0, /*gap_behind=*/true, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 4.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 8.5, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 4.0, 1e-3);
  }
  {
    /*
    ------               ego_v
    ======\============  target_v
           \    -------  final_v
            \  /
             \/          mid_v
    ===================  align_v (not used)
    */
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/1.5, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/7.0, /*gap_behind=*/true, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 5.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 23.5, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 4.0, 1e-3);
  }
  {
    /*
     ------                         ego_v
     ======\======================  target_v
            \               ------  final_v
             \             /
     =========-------------=======  align_v
     */
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/18.5, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/7.0, /*gap_behind=*/true, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 11.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 36.5, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 4.0, 1e-3);
  }
  {
    /*
     =========================  target_v
                        ------  final_v
                       /
     ------           /         ego_v
           \         /
     =======---------=========  align_v
     */
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/18.5, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/3.0, /*gap_behind=*/true, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 7.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 16.5, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 4.0, 1e-3);
  }
}

TEST(TrafficGapFinder, ComputeGapAlignInfoWithGapAhead) {
  {
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/22.5, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/12.0, /*gap_behind=*/false, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 5.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 47.5, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 7.0, 1e-3);
  }
  {
    /*
    --------------------          ego_v
    ====================\=======  align_v (not used)
                         \
                          ------  final_v
    ============================  target_v
    */
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/22.0, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/10.0, /*gap_behind=*/false, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 6.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 52.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 6.0, 1e-3);
  }
  {
    /*
    ===================  align_v
             /\          mid_v
    ---------  \         ego_v
                -------  final_v
    ===================  target_v
    */
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/3.75, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/7.0, /*gap_behind=*/false, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 2.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 13.75, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 6.0, 1e-3);
  }
  {
    /*
    ===================  align_v

    ------------         ego_v, cap_v, mid_v
                -------  final_v
    ===================  target_v
    */
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/3.75, /*target_v=*/5.0, /*cap_v=*/7.0,
        /*ego_v=*/7.0, /*gap_behind=*/false, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 2.125, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 14.375, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 6.0, 1e-3);
  }
  {
    /*
     =======---------=========  align_v
           /         \
     ------           \         ego_v
                       -------  final_v
     =========================  target_v
    */
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/15.5, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/7.0, /*gap_behind=*/false, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 6.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 45.5, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 6.0, 1e-3);
  }
  {
    /*
    =========-------------=======  align_v
            /             \
           /               ------  final_v
    ======/======================  target_v
    ------                         ego_v
    */
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/12.5, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/3.0, /*gap_behind=*/false, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 9.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 57.5, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 6.0, 1e-3);
  }
  {
    /*
    =============================  align_v
            ---------------        cap_v
           /               ------  final_v
    ======/======================  target_v
    ------                         ego_v
    */
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/12.5, /*target_v=*/5.0, /*cap_v=*/7.0,
        /*ego_v=*/3.0, /*gap_behind=*/false, /*gap_aligned=*/false);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 10.5, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 65.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 6.0, 1e-3);
  }
}

TEST(TrafficGapFinder, ComputeAlignTimeAndDistWithAlignedGap) {
  {
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/20.0, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/10.0, /*gap_behind=*/false, /*gap_aligned=*/true);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 0.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 0.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 10.0, 1e-3);
  }
  {
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/2.0, /*target_v=*/10.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/8.0, /*gap_behind=*/false, /*gap_aligned=*/true);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 3.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 28.5, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 11.0, 1e-3);
  }
  {
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/9.0, /*target_v=*/10.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/5.0, /*gap_behind=*/false, /*gap_aligned=*/true);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 8.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 71.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 11.0, 1e-3);
  }
  {
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/1.0, /*target_v=*/10.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/5.0, /*gap_behind=*/false, /*gap_aligned=*/true);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 11.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 109.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 11.0, 1e-3);
  }
  {
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/20.0, /*target_v=*/10.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/5.0, /*gap_behind=*/true, /*gap_aligned=*/true);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 0.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 0.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 5.0, 1e-3);
  }
  {
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/2.0, /*target_v=*/8.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/10.0, /*gap_behind=*/true, /*gap_aligned=*/true);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 3.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 25.5, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 7.0, 1e-3);
  }
  {
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/9.0, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/10.0, /*gap_behind=*/true, /*gap_aligned=*/true);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 8.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 49.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 4.0, 1e-3);
  }
  {
    const auto align_info_or = ComputeGapAlignInfo(
        /*lon_offset=*/1.0, /*target_v=*/5.0, /*cap_v=*/DBL_MAX,
        /*ego_v=*/10.0, /*gap_behind=*/true, /*gap_aligned=*/true);
    ASSERT_OK(align_info_or);
    EXPECT_NEAR(align_info_or->align_time, 11.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_s, 56.0, 1e-3);
    EXPECT_NEAR(align_info_or->align_v, 4.0, 1e-3);
  }
}

TEST(TrafficGapFinder, FindNearestLeadingObject) {
  const auto& psmm = CreateDojoTestPSMM();
  const mapping::LanePath target_lane_path = *BuildLanePathFromData(
      mapping::LanePathData(
          /*start_fraction=*/0.0, /*end_fraction=*/1.0,
          {mapping::ElementId(2476), mapping::ElementId(2491),
           mapping::ElementId(2484)}),
      psmm);
  const auto frenet_frame =
      *BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, target_lane_path),
                              /*down_sample_raw_points=*/true);

  ObjectVector<PlannerObject> objects;
  objects.push_back(
      BuildPlannerObject("leading", Vec2d(161.7, 66.5), /*vel=*/7.0));
  objects.push_back(BuildPlannerObject("filtered", Vec2d(150.0, 66.0),
                                       /*vel=*/5.0, /*heading=*/-1.0));
  const PlannerObjectManager obj_mgr(objects);
  const SpacetimeTrajectoryManager st_traj_mgr(obj_mgr.planner_objects());
  DrawPlannerObjectManagerToCanvas(obj_mgr, "gap_finder/cap_speed/objects",
                                   vis::Color::kLightGreen);

  const Box2d ego_box(/*half_length=*/2.1, /*half_width=*/1.05,
                      Vec2d(130.0, 66.5), /*heading=*/0.0);
  SendBox2dToCanvas(ego_box, "gap_finder/cap_speed/ego_vehicle",
                    vis::Color::kOrange);
  const auto ego_frenet_box = *frenet_frame.QueryFrenetBoxAt(ego_box);

  const double ego_v = 6.0;
  const auto [cap_v, lon_offset] = FindNearestLeadingObject(
      frenet_frame, ego_frenet_box, ego_v, st_traj_mgr);
  EXPECT_NEAR(cap_v, 7.0, 1e-3);
  EXPECT_NEAR(lon_offset, 24.5, 1e-2);
}

}  // namespace

}  // namespace qcraft::planner
