#include "onboard/planner/speed/st_boundary.h"

#include <initializer_list>

#include "absl/functional/bind_front.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"

namespace qcraft {
namespace planner {
namespace {

using ::testing::AllOf;
using ::testing::DoubleNear;
using ::testing::ElementsAre;
using ::testing::ElementsAreArray;
using ::testing::Eq;
using ::testing::ExplainMatchResult;
using ::testing::Field;
using ::testing::NotNull;
using ::testing::Optional;
using ::testing::Pointwise;
using ::testing::ResultOf;
using ::testing::StrEq;

template <typename ArgType, typename ResultType, typename Transformer>
void ExpectEach(const std::initializer_list<const ArgType>& args,
                const std::initializer_list<const ResultType>& results,
                const Transformer& transformer) {
  ASSERT_EQ(args.size(), results.size());
  auto result_iter = results.begin();
  for (auto arg_iter = args.begin(); arg_iter != args.end(); ++arg_iter) {
    EXPECT_THAT(*arg_iter, ResultOf(transformer, *result_iter));
    ++result_iter;
  }
}

MATCHER_P(StPointIs, st, "") {
  *result_listener << arg.DebugString();
  return std::abs(st.s() - arg.s()) < 1e-3 && std::abs(st.t() - arg.t()) < 1e-3;
}

MATCHER_P2(StAre, s, t, "") {
  *result_listener << arg.DebugString();
  return std::abs(s - arg.s()) < 1e-3 && std::abs(t - arg.t()) < 1e-3;
}

StBoundaryPoints BuildUpRightStBoundaryPoints() {
  StBoundaryPoints st_boundary_points;
  constexpr double kStartT = 3.0;
  constexpr double kStartS = 20.0;
  constexpr double kOffsetS = 2.5;
  constexpr double kStepT = 0.1;
  constexpr double kStepS = 1.0;
  const double v = kStepS / kStepT;
  constexpr int kNum = 10;
  for (int i = 0; i < kNum; ++i) {
    const double t = kStartT + kStepT * i;
    const double center_s = kStartS + kStepS * i;
    st_boundary_points.lower_points.push_back(StPoint(center_s - kOffsetS, t));
    st_boundary_points.upper_points.push_back(StPoint(center_s + kOffsetS, t));
    st_boundary_points.speed_points.push_back(VtPoint(v, t));
  }
  st_boundary_points.overlap_infos.push_back({
      .time = kStartT,
      .obj_idx = 10,
      .av_start_idx = 20,
      .av_end_idx = 30,
  });
  return st_boundary_points;
}

StBoundaryPoints BuildDownRightStBoundaryPoints() {
  StBoundaryPoints st_boundary_points;
  constexpr double kStartT = 3.0;
  constexpr double kStartS = 20.0;
  constexpr double kOffsetS = 2.5;
  constexpr double kStepT = 0.1;
  constexpr double kStepS = -1.0;
  const double v = kStepS / kStepT;
  constexpr int kNum = 10;
  for (int i = 0; i < kNum; ++i) {
    const double t = kStartT + kStepT * i;
    const double center_s = kStartS + kStepS * i;
    st_boundary_points.lower_points.push_back(StPoint(center_s - kOffsetS, t));
    st_boundary_points.upper_points.push_back(StPoint(center_s + kOffsetS, t));
    st_boundary_points.speed_points.push_back(VtPoint(v, t));
  }
  st_boundary_points.overlap_infos.push_back({
      .time = kStartT,
      .obj_idx = 10,
      .av_start_idx = 20,
      .av_end_idx = 30,
  });
  return st_boundary_points;
}

StBoundaryPoints BuildFlatStBoundaryPoints() {
  StBoundaryPoints st_boundary_points;
  constexpr double kStartT = 0.0;
  constexpr double kStartS = 20.0;
  constexpr double kOffsetS = 5.0;
  constexpr double kStepT = 0.1;
  constexpr double kStepS = 0.0;
  const double v = kStepS / kStepT;
  constexpr int kNum = 60;
  for (int i = 0; i < kNum; ++i) {
    const double t = kStartT + kStepT * i;
    const double center_s = kStartS + kStepS * i;
    st_boundary_points.lower_points.push_back(StPoint(center_s - kOffsetS, t));
    st_boundary_points.upper_points.push_back(StPoint(center_s + kOffsetS, t));
    st_boundary_points.speed_points.push_back(VtPoint(v, t));
  }
  st_boundary_points.overlap_infos.push_back({
      .time = kStartT,
      .obj_idx = 0,
      .av_start_idx = 10,
      .av_end_idx = 30,
  });
  return st_boundary_points;
}
TEST(StBoundary, RecoverObjectId) {
  EXPECT_THAT(StBoundary::RecoverObjectId("abc-idx1",
                                          StBoundarySourceTypeProto::VIRTUAL),
              Eq(std::nullopt));
  EXPECT_THAT(
      StBoundary::RecoverObjectId("abc", StBoundarySourceTypeProto::ST_OBJECT),
      Eq(std::nullopt));
  EXPECT_THAT(StBoundary::RecoverObjectId("abc-idx1",
                                          StBoundarySourceTypeProto::ST_OBJECT),
              Optional(std::string("abc")));
  EXPECT_THAT(
      StBoundary::RecoverObjectId("idx1", StBoundarySourceTypeProto::ST_OBJECT),
      Eq(std::nullopt));
}

TEST(StBoundary, RecoverTrajId) {
  EXPECT_THAT(
      StBoundary::RecoverTrajId("abc-idx1", StBoundarySourceTypeProto::VIRTUAL),
      Eq(std::nullopt));
  EXPECT_THAT(StBoundary::RecoverTrajId("abc-idx1",
                                        StBoundarySourceTypeProto::ST_OBJECT),
              Optional(std::string("abc-idx1")));
  EXPECT_THAT(StBoundary::RecoverTrajId("abc|idx1",
                                        StBoundarySourceTypeProto::ST_OBJECT),
              Optional(std::string("abc")));
}

TEST(StBoundary, UpRightStBoundary) {
  const auto st_boundary_points = BuildUpRightStBoundaryPoints();
  const auto st_boundary = StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VEHICLE, "vehicle-idx0",
      /*probability=*/0.5,
      /*is_stationary=*/false, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
  ASSERT_THAT(st_boundary, NotNull());

  EXPECT_FALSE(st_boundary->IsEmpty());

  ExpectEach(
      {StPoint(0.0, 0.0), StPoint(25, 3.5), StPoint(30, 5.0), StPoint(50, 3.5),
       StPoint(0, 3.5)},
      {false, true, false, false, false},
      absl::bind_front(&StBoundary::IsPointInBoundary, st_boundary.get()));

  EXPECT_THAT(st_boundary->upper_left_point(), StAre(22.5, 3.0));
  EXPECT_THAT(st_boundary->bottom_left_point(), StAre(17.5, 3.0));
  EXPECT_THAT(st_boundary->upper_right_point(), StAre(31.5, 3.9));
  EXPECT_THAT(st_boundary->bottom_right_point(), StAre(26.5, 3.9));

  EXPECT_EQ(st_boundary->source_type(), StBoundarySourceTypeProto::ST_OBJECT);
  EXPECT_EQ(st_boundary->object_type(), StBoundaryProto::VEHICLE);
  EXPECT_EQ(st_boundary->id(), "vehicle-idx0");
  EXPECT_THAT(st_boundary->traj_id(), Optional(std::string("vehicle-idx0")));
  EXPECT_THAT(st_boundary->object_id(), Optional(std::string("vehicle")));
  EXPECT_THAT(st_boundary->probability(), 0.5);
  EXPECT_FALSE(st_boundary->is_stationary());
  EXPECT_FALSE(st_boundary->is_protective());
  EXPECT_EQ(st_boundary->protection_type(), StBoundaryProto::NON_PROTECTIVE);
  EXPECT_THAT(st_boundary->protected_st_boundary_id(), Eq(std::nullopt));
  EXPECT_FALSE(st_boundary->is_large_vehicle());
  // EXPECT_THAT(st_boundary.GetBoundarySRange());

  EXPECT_NEAR(st_boundary->min_s(), 17.5, 1e-3);
  EXPECT_NEAR(st_boundary->max_s(), 31.5, 1e-3);
  EXPECT_NEAR(st_boundary->min_t(), 3.0, 1e-3);
  EXPECT_NEAR(st_boundary->max_t(), 3.9, 1e-3);

  EXPECT_THAT(st_boundary->upper_points(),
              ElementsAre(StPoint(22.5, 3.0), StPoint(31.5, 3.9)));
  EXPECT_THAT(st_boundary->lower_points(),
              ElementsAre(StPoint(17.5, 3.0), StPoint(26.5, 3.9)));
  EXPECT_THAT(st_boundary->speed_points(),
              ElementsAreArray(st_boundary_points.speed_points));

  EXPECT_THAT(st_boundary->overlap_infos(),
              ElementsAre(AllOf(Field(&OverlapInfo::time, 3.0),
                                Field(&OverlapInfo::obj_idx, 10),
                                Field(&OverlapInfo::av_start_idx, 20),
                                Field(&OverlapInfo::av_end_idx, 30))));
}

TEST(StBoundary, DownRightStBoundary) {
  const auto st_boundary_points = BuildDownRightStBoundaryPoints();
  const auto st_boundary = StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VEHICLE, "vehicle-idx0",
      /*probability=*/0.5,
      /*is_stationary=*/false, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
  ASSERT_THAT(st_boundary, NotNull());

  EXPECT_FALSE(st_boundary->IsEmpty());

  ExpectEach(
      {StPoint(0.0, 0.0), StPoint(20, 3.1), StPoint(30, 5.0), StPoint(50, 3.5),
       StPoint(0, 3.5)},
      {false, true, false, false, false},
      absl::bind_front(&StBoundary::IsPointInBoundary, st_boundary.get()));

  EXPECT_THAT(st_boundary->upper_left_point(), StAre(22.5, 3.0));
  EXPECT_THAT(st_boundary->bottom_left_point(), StAre(17.5, 3.0));
  EXPECT_THAT(st_boundary->upper_right_point(), StAre(13.5, 3.9));
  EXPECT_THAT(st_boundary->bottom_right_point(), StAre(8.5, 3.9));

  EXPECT_EQ(st_boundary->source_type(), StBoundarySourceTypeProto::ST_OBJECT);
  EXPECT_EQ(st_boundary->object_type(), StBoundaryProto::VEHICLE);
  EXPECT_EQ(st_boundary->id(), "vehicle-idx0");
  EXPECT_THAT(st_boundary->traj_id(), Optional(std::string("vehicle-idx0")));
  EXPECT_THAT(st_boundary->object_id(), Optional(std::string("vehicle")));
  EXPECT_THAT(st_boundary->probability(), 0.5);
  EXPECT_FALSE(st_boundary->is_stationary());
  EXPECT_FALSE(st_boundary->is_protective());
  EXPECT_EQ(st_boundary->protection_type(), StBoundaryProto::NON_PROTECTIVE);
  EXPECT_THAT(st_boundary->protected_st_boundary_id(), Eq(std::nullopt));
  EXPECT_FALSE(st_boundary->is_large_vehicle());
  // EXPECT_THAT(st_boundary.GetBoundarySRange());

  EXPECT_NEAR(st_boundary->min_s(), 8.5, 1e-3);
  EXPECT_NEAR(st_boundary->max_s(), 22.5, 1e-3);
  EXPECT_NEAR(st_boundary->min_t(), 3.0, 1e-3);
  EXPECT_NEAR(st_boundary->max_t(), 3.9, 1e-3);

  EXPECT_THAT(st_boundary->upper_points(),
              ElementsAre(StPoint{22.5, 3.0}, StPoint{13.5, 3.9}));
  EXPECT_THAT(st_boundary->lower_points(),
              ElementsAre(StPoint{17.5, 3.0}, StPoint{8.5, 3.9}));
  EXPECT_THAT(st_boundary->speed_points(),
              ElementsAreArray(st_boundary_points.speed_points));

  EXPECT_THAT(st_boundary->overlap_infos(),
              ElementsAre(AllOf(Field(&OverlapInfo::time, 3.0),
                                Field(&OverlapInfo::obj_idx, 10),
                                Field(&OverlapInfo::av_start_idx, 20),
                                Field(&OverlapInfo::av_end_idx, 30))));
}

TEST(StBoundary, FlatStBoundary) {
  const auto st_boundary_points = BuildFlatStBoundaryPoints();
  const auto st_boundary = StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VEHICLE, "vehicle-idx0",
      /*probability=*/0.5,
      /*is_stationary=*/false, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
  ASSERT_THAT(st_boundary, NotNull());

  EXPECT_FALSE(st_boundary->IsEmpty());

  ExpectEach(
      {StPoint(0.0, 0.0), StPoint(20, 3.5), StPoint(30, 5.0), StPoint(50, 3.5),
       StPoint(0, 3.5)},
      {false, true, false, false, false},
      absl::bind_front(&StBoundary::IsPointInBoundary, st_boundary.get()));

  EXPECT_THAT(st_boundary->upper_left_point(), StAre(25, 0.0));
  EXPECT_THAT(st_boundary->bottom_left_point(), StAre(15.0, 0.0));
  EXPECT_THAT(st_boundary->upper_right_point(), StAre(25.0, 5.9));
  EXPECT_THAT(st_boundary->bottom_right_point(), StAre(15.0, 5.9));

  EXPECT_EQ(st_boundary->source_type(), StBoundarySourceTypeProto::ST_OBJECT);
  EXPECT_EQ(st_boundary->object_type(), StBoundaryProto::VEHICLE);
  EXPECT_EQ(st_boundary->id(), "vehicle-idx0");
  EXPECT_THAT(st_boundary->traj_id(), Optional(std::string("vehicle-idx0")));
  EXPECT_THAT(st_boundary->object_id(), Optional(std::string("vehicle")));
  EXPECT_THAT(st_boundary->probability(), 0.5);
  EXPECT_FALSE(st_boundary->is_stationary());
  EXPECT_FALSE(st_boundary->is_protective());
  EXPECT_EQ(st_boundary->protection_type(), StBoundaryProto::NON_PROTECTIVE);
  EXPECT_THAT(st_boundary->protected_st_boundary_id(), Eq(std::nullopt));
  EXPECT_FALSE(st_boundary->is_large_vehicle());
  // EXPECT_THAT(st_boundary.GetBoundarySRange());

  EXPECT_NEAR(st_boundary->min_s(), 15, 1e-3);
  EXPECT_NEAR(st_boundary->max_s(), 25, 1e-3);
  EXPECT_NEAR(st_boundary->min_t(), 0.0, 1e-3);
  EXPECT_NEAR(st_boundary->max_t(), 5.9, 1e-3);

  EXPECT_THAT(st_boundary->upper_points(),
              ElementsAre(StPoint{25.0, 0.0}, StPoint{25.0, 5.9}));
  EXPECT_THAT(st_boundary->lower_points(),
              ElementsAre(StPoint{15.0, 0.0}, StPoint{15.0, 5.9}));
  EXPECT_THAT(st_boundary->speed_points(),
              ElementsAreArray(st_boundary_points.speed_points));

  EXPECT_THAT(st_boundary->overlap_infos(),
              ElementsAre(AllOf(Field(&OverlapInfo::time, 0.0),
                                Field(&OverlapInfo::obj_idx, 0),
                                Field(&OverlapInfo::av_start_idx, 10),
                                Field(&OverlapInfo::av_end_idx, 30))));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
