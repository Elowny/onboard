#include "onboard/planner/common/path_approx.h"

#include <cmath>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/test_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {
namespace planner {
namespace {

using ::testing::ElementsAre;

TEST(PathSegment, SimpleTest) {
  const PathSegment seg(1, 10, Vec2d(0.0, 1.0), Vec2d(2.0, 3.0),
                        /*first_s=*/1.0, /*last_s=*/2.0,
                        Box2d(Vec2d(1.0, 2.0), /*heading=*/M_PI * 0.25,
                              /*length=*/5.0, /*width=*/2.0));
  EXPECT_EQ(seg.first_index(), 1);
  EXPECT_EQ(seg.last_index(), 10);
  EXPECT_EQ(seg.first_s(), 1.0);
  EXPECT_EQ(seg.last_s(), 2.0);
  EXPECT_THAT(seg.first_ra(), Vec2dEqXY(0.0, 1.0));
  EXPECT_THAT(seg.last_ra(), Vec2dEqXY(2.0, 3.0));
}

TEST(PathApprox, SimpleTest) {
  PathPoint p0, p1, p2, p3;
  TextToProto(R"(x: 0.0 y: 0.0 theta: 0.0, s: 0.0)", &p0);
  TextToProto(R"(x: 1.0 y: 0.0 theta: 0.0, s: 1.0)", &p1);
  TextToProto(R"(x: 2.0 y: 0.0 theta: 0.1, s: 2.0)", &p2);
  TextToProto(R"(x: 2.0 y: 1.0 theta: 1.7, s: 3.0)", &p3);

  const auto vehicle_geom = DefaultVehicleGeometry();

  const OffsetRect rect = CreateOffsetRectFromVehicleGeometry(vehicle_geom);
  constexpr double kPadding = 1.0;
  const OffsetRect padded_rect(rect.half_length(), rect.half_width() + kPadding,
                               rect.offset());

  const auto path1 =
      BuildPathApprox({p0, p1}, padded_rect,
                      /*tolerance=*/0.01, /*path_kd_tree=*/nullptr);
  EXPECT_EQ(path1.segments().size(), 1);
  EXPECT_NEAR(path1.segments()[0].length(), vehicle_geom.length() + 1.0, 1e-4);
  EXPECT_THAT(path1.segments()[0].tangent(), Vec2dNearXY(1.0, 0.0, 1e-4));
  EXPECT_NEAR(path1.segments()[0].width(), vehicle_geom.width() + 2.0, 1e-4);
  EXPECT_NEAR(path1.segments()[0].first_s(), 0.0, 1e-4);
  EXPECT_NEAR(path1.segments()[0].last_s(), 1.0, 1e-4);
  EXPECT_THAT(path1.point_to_segment_index(), ElementsAre(0, 0));

  const auto path2 =
      BuildPathApprox({p0, p1, p2}, padded_rect,
                      /*tolerance=*/0.01, /*path_kd_tree=*/nullptr);
  EXPECT_EQ(path2.segments().size(), 2);
  EXPECT_NEAR(path2.segments()[0].length(), vehicle_geom.length() + 1.0, 1e-4);
  EXPECT_NEAR(path2.segments()[0].width(), vehicle_geom.width() + 2.0, 1e-4);
  EXPECT_NEAR(path2.segments()[0].heading(), 0.0, 1e-4);

  EXPECT_NEAR(path2.segments()[1].width(), vehicle_geom.width() + 2.0, 1e-4);
  EXPECT_NEAR(path2.segments()[1].length(), vehicle_geom.length(), 1e-4);
  EXPECT_NEAR(path2.segments()[1].heading(), 0.1, 1e-4);
  EXPECT_THAT(path2.point_to_segment_index(), ElementsAre(0, 0, 1));

  const auto path3 =
      BuildPathApprox({p0, p1, p2, p3}, padded_rect,
                      /*tolerance=*/0.01, /*path_kd_tree=*/nullptr);
  EXPECT_EQ(path3.segments().size(), 3);
  EXPECT_NEAR(path3.segments()[0].length(), vehicle_geom.length() + 1.0, 1e-4);
  EXPECT_NEAR(path3.segments()[0].width(), vehicle_geom.width() + 2.0, 1e-4);
  EXPECT_NEAR(path3.segments()[0].heading(), 0.0, 1e-4);

  EXPECT_NEAR(path3.segments()[1].width(), vehicle_geom.width() + 2.0, 1e-4);
  EXPECT_NEAR(path3.segments()[1].length(), vehicle_geom.length(), 1e-4);
  EXPECT_NEAR(path3.segments()[1].heading(), 0.1, 1e-4);

  EXPECT_NEAR(path3.segments()[2].width(), vehicle_geom.width() + 2.0, 1e-4);
  EXPECT_NEAR(path3.segments()[2].length(), vehicle_geom.length(), 1e-4);
  EXPECT_NEAR(path3.segments()[2].heading(), 1.7, 1e-4);
  EXPECT_NEAR(path3.segments()[2].first_s(), 3.0, 1e-4);
  EXPECT_NEAR(path3.segments()[2].last_s(), 3.0, 1e-4);
  EXPECT_THAT(path3.point_to_segment_index(), ElementsAre(0, 0, 1, 2));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
