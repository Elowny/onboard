#include "onboard/prediction/predictor/predictor_util.h"

#include <string>
#include <unordered_set>  // for unordered_set

#include "gtest/gtest.h"

#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/proto/perception.pb.h"  // for ObjectProto

namespace qcraft {
namespace prediction {
namespace {
TEST(PredictorUtilTest, ScreenPredictObjectsByDistance) {
  auto obj_1 = BuildVehicleHistoryByConstVel("obj_1", 10, Vec2d(3.0, 4.0), 5.0);
  auto obj_2 =
      BuildVehicleHistoryByConstVel("obj_2", 10, Vec2d(10.0, 0.0), 6.0);
  std::vector<ObjectHistory*> objects_history;
  objects_history.reserve(2);
  objects_history.push_back(&obj_1);
  objects_history.push_back(&obj_2);
  const auto ego_box = Box2d(Vec2d::Zero(), 0.0, 4.0, 2.0);
  const auto screen_results_1 =
      ScreenPredictObjectsByDistance(ego_box, objects_history, 1);
  EXPECT_EQ(screen_results_1.size(), 1);
  EXPECT_EQ(screen_results_1[0]->id(), "obj_1");
  const auto screen_results_2 =
      ScreenPredictObjectsByDistance(ego_box, objects_history, 2);
  EXPECT_EQ(screen_results_2.size(), 2);
  EXPECT_EQ(screen_results_2[0]->id(), "obj_1");
}

TEST(PredictorUtilTest, GetCurrentTimeStamp) {
  const std::string id = "object_0";
  const double vel = 5.0;
  auto object_1s = BuildVehicleHistoryByConstVel(id, 10, Vec2d::Zero(), vel);
  auto object_2s = BuildVehicleHistoryByConstVel(id, 20, Vec2d::Zero(), vel);
  std::vector<ObjectHistory*> objects_history;
  objects_history.reserve(2);
  EXPECT_NEAR(GetCurrentTimeStamp(0.5, objects_history), 0.5, 1e-6);
  objects_history.push_back(&object_1s);
  objects_history.push_back(&object_2s);
  EXPECT_NEAR(GetCurrentTimeStamp(0.5, objects_history), 1.4, 1e-6);
}
TEST(PredictorUtilTest, Vec2dPointsToPredTrajPoints) {
  std::vector<Vec2d> vec2d_points = {Vec2d::Zero(),   Vec2d(3.0, 4.0),
                                     Vec2d(5.0, 4.0), Vec2d(5.0, 6.0),
                                     Vec2d(6.0, 8.0), Vec2d(6.0, 10.0)};
  std::vector<double> vec_t = {10.0, 10.5, 10.6, 10.7, 10.8, 10.9};
  const auto pred_points_1 = Vec2dPointsToPredTrajPoints(
      vec2d_points, vec_t, 10, /*use_pos_fitter=*/false,
      /*polyfit_downsample_step=*/1);
  EXPECT_EQ(pred_points_1.size(), 10);
  const auto pred_points_2 = Vec2dPointsToPredTrajPoints(
      vec2d_points, vec_t, 20, /*use_pos_fitter=*/true,
      /*polyfit_downsample_step=*/1);
  EXPECT_EQ(pred_points_2.size(), 20);
}
TEST(PredictorUtilTest, PostProcessOneTrajPoint) {
  std::vector<PredictedTrajectoryPoint> traj_points = {
      PredictedTrajectoryPoint()};
  const auto obj = planner::PerceptionObjectBuilder().Build();
  ObjectMotionState state{
      .timestamp = obj.timestamp(),
      .pos = Vec2d(obj.pos()),
      .heading = obj.yaw(),
      .vel = Vec2d(obj.vel()),
      .bbox = Box2d(obj.bounding_box()),
  };
  const auto processed_pts = PostProcessModelOutputTrajPts(
      traj_points, state, 0.1, 10.0, /*perception_acc=*/0.0,
      /*rectify_speed=*/false);
  EXPECT_EQ(processed_pts.size(), 1);
}
TEST(PredictorUtilTest, IsObjectInRange) {
  const Vec2d object_pos(3.0, 4.0);
  const Vec2d center(0.0, 0.0);
  const double in_radius = 1.0, out_radius = 6.0;
  EXPECT_FALSE(IsObjectInRange(object_pos, center, in_radius));
  EXPECT_TRUE(IsObjectInRange(object_pos, center, out_radius));
}

TEST(PredictorUtilTest, IsBicycleModelLike) {
  EXPECT_TRUE(IsBicycleModelLike(OT_VEHICLE));
  EXPECT_TRUE(IsBicycleModelLike(OT_CYCLIST));
  EXPECT_TRUE(IsBicycleModelLike(OT_TRICYCLIST));
  EXPECT_TRUE(IsBicycleModelLike(OT_MOTORCYCLIST));
  EXPECT_TRUE(IsBicycleModelLike(OT_UNKNOWN_MOVABLE));
  EXPECT_FALSE(IsBicycleModelLike(OT_UNKNOWN_STATIC));
  EXPECT_FALSE(IsBicycleModelLike(OT_FOD));
  EXPECT_FALSE(IsBicycleModelLike(OT_VEGETATION));
  EXPECT_FALSE(IsBicycleModelLike(OT_BARRIER));
  EXPECT_FALSE(IsBicycleModelLike(OT_CONE));
  EXPECT_FALSE(IsBicycleModelLike(OT_WARNING_TRIANGLE));
  EXPECT_FALSE(IsBicycleModelLike(OT_PEDESTRIAN));
}

TEST(PredictorUtilTest, NormalizeAndDescSortTrajProbs) {
  std::vector<PredictedTrajectory> pred_trajs;
  pred_trajs.reserve(2);
  pred_trajs.emplace_back();
  NormalizeAndDescSortTrajProbs(absl::MakeSpan(pred_trajs));
  EXPECT_EQ(pred_trajs.size(), 1);
  EXPECT_EQ(pred_trajs[0].probability(), 0.0);
  pred_trajs.back().set_probability(0.6);
  pred_trajs.emplace_back();
  pred_trajs.back().set_probability(0.9);
  NormalizeAndDescSortTrajProbs(absl::MakeSpan(pred_trajs));
  EXPECT_EQ(pred_trajs.size(), 2);
  EXPECT_NEAR(pred_trajs[0].probability(), 0.6, 1e-4);
  EXPECT_NEAR(pred_trajs[1].probability(), 0.4, 1e-4);
}

TEST(PredictorUtilTest, SelectPredictedObjectsByTypePriority) {
  std::vector<ObjectHistory> objects_history_veh;
  std::vector<ObjectHistory> objects_history_cyc;
  std::vector<ObjectHistory> objects_history_unk;
  std::vector<ObjectHistory*> objects_history_ptrs;
  objects_history_veh.reserve(8);
  objects_history_cyc.reserve(8);
  objects_history_unk.reserve(8);
  objects_history_ptrs.reserve(24);
  for (int i = 0; i < 8; ++i) {
    objects_history_veh.push_back(BuildHistoryByConstVel(
        "veh_" + std::to_string(i), 10, Vec2d(5 + i, 0.0), 6.0, OT_VEHICLE));
    objects_history_cyc.push_back(BuildHistoryByConstVel(
        "cyc_" + std::to_string(i), 10, Vec2d(5 + i, 0.0), 6.0, OT_CYCLIST));
    objects_history_unk.push_back(
        BuildHistoryByConstVel("unk_" + std::to_string(i), 10,
                               Vec2d(5 + i, 0.0), 6.0, OT_UNKNOWN_MOVABLE));
  }

  const auto ego_box = Box2d(Vec2d::Zero(), 0.0, 4.0, 2.0);
  const int max_pred_num = 6;
  const TypePrioMap type_prio_map = {
      {PredictTypePrio::HIGH, {OT_VEHICLE}},
      {PredictTypePrio::MED, {OT_CYCLIST}},
      {PredictTypePrio::LOW, {OT_UNKNOWN_MOVABLE}}};
  const std::map<PredictTypePrio, int> type_max_num_map = {
      {PredictTypePrio::HIGH, 2},
      {PredictTypePrio::MED, 2},
      {PredictTypePrio::LOW, 2}};

  auto check_func =
      [&](const std::map<PredictTypePrio, int>& type_origin_num_map,
          const std::map<PredictTypePrio, int>& type_candidate_num_map) {
        objects_history_ptrs.clear();
        for (int i = 0; i < type_origin_num_map.at(PredictTypePrio::HIGH);
             ++i) {
          objects_history_ptrs.push_back(&objects_history_veh[i]);
        }
        for (int i = 0; i < type_origin_num_map.at(PredictTypePrio::MED); ++i) {
          objects_history_ptrs.push_back(&objects_history_cyc[i]);
        }
        for (int i = 0; i < type_origin_num_map.at(PredictTypePrio::LOW); ++i) {
          objects_history_ptrs.push_back(&objects_history_unk[i]);
        }
        const auto candidate_objs = SelectPredictedObjectsByTypePriority(
            ego_box, max_pred_num, objects_history_ptrs, type_prio_map,
            type_max_num_map);
        // Check results.
        std::unordered_set<std::string> candidate_objs_id_set;
        for (const auto* candidate_obj : candidate_objs) {
          candidate_objs_id_set.insert(candidate_obj->id());
        }

        for (int i = 0; i < type_candidate_num_map.at(PredictTypePrio::HIGH);
             ++i) {
          EXPECT_TRUE(candidate_objs_id_set.count("veh_" + std::to_string(i)));
        }
        for (int i = 0; i < type_candidate_num_map.at(PredictTypePrio::MED);
             ++i) {
          EXPECT_TRUE(candidate_objs_id_set.count("cyc_" + std::to_string(i)));
        }
        for (int i = 0; i < type_candidate_num_map.at(PredictTypePrio::LOW);
             ++i) {
          EXPECT_TRUE(candidate_objs_id_set.count("unk_" + std::to_string(i)));
        }
      };

  // Origin number: 1, 1, 1 -> Candidate number: 1, 1, 1
  {
    const std::map<PredictTypePrio, int> type_origin_num_map = {
        {PredictTypePrio::HIGH, 1},
        {PredictTypePrio::MED, 1},
        {PredictTypePrio::LOW, 1}};
    const std::map<PredictTypePrio, int> type_candidate_num_map = {
        {PredictTypePrio::HIGH, 1},
        {PredictTypePrio::MED, 1},
        {PredictTypePrio::LOW, 1}};
    check_func(type_origin_num_map, type_candidate_num_map);
  }
  // Origin number: 4, 3, 1 -> Candidate number: 3, 2, 1
  {
    const std::map<PredictTypePrio, int> type_origin_num_map = {
        {PredictTypePrio::HIGH, 4},
        {PredictTypePrio::MED, 3},
        {PredictTypePrio::LOW, 1}};
    const std::map<PredictTypePrio, int> type_candidate_num_map = {
        {PredictTypePrio::HIGH, 3},
        {PredictTypePrio::MED, 2},
        {PredictTypePrio::LOW, 1}};
    check_func(type_origin_num_map, type_candidate_num_map);
  }
  // Origin number: 7, 0, 1 -> Candidate number: 5, 0, 1
  {
    const std::map<PredictTypePrio, int> type_origin_num_map = {
        {PredictTypePrio::HIGH, 7},
        {PredictTypePrio::MED, 0},
        {PredictTypePrio::LOW, 1}};
    const std::map<PredictTypePrio, int> type_candidate_num_map = {
        {PredictTypePrio::HIGH, 5},
        {PredictTypePrio::MED, 0},
        {PredictTypePrio::LOW, 1}};
    check_func(type_origin_num_map, type_candidate_num_map);
  }
  // Origin number: 1, 0, 7 -> Candidate number: 1, 0, 5
  {
    const std::map<PredictTypePrio, int> type_origin_num_map = {
        {PredictTypePrio::HIGH, 1},
        {PredictTypePrio::MED, 0},
        {PredictTypePrio::LOW, 7}};
    const std::map<PredictTypePrio, int> type_candidate_num_map = {
        {PredictTypePrio::HIGH, 1},
        {PredictTypePrio::MED, 0},
        {PredictTypePrio::LOW, 5}};
    check_func(type_origin_num_map, type_candidate_num_map);
  }
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
