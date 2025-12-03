#include "onboard/prediction/predicted_trajectory.h"

#include "google/protobuf/text_format.h"

#include "gtest/gtest.h"

#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
namespace {

PredictedTrajectoryProto SamplePredictedTrajectoryProto() {
  PredictedTrajectoryProto proto;
  std::string text_proto = R"(
    probability: 0.6
    annotation: "something"
    type: PT_ACTNET
    index: 0
    is_reversed: false
    av_relation: OR_YIELD
    av_relation_prob {
      pass: 0.1
      yield: 0.8
      no_relation: 0.1
    }
    relation_info {
      av_relation: OR_PASS
      av_s: 10.0
      object_s: 15.0
    }
    points {
      pos {
       x: 1.0
       y: 2.0
      }
      s: 0.0
      t: 0.0
      a: 0.0
      v: 1.0
    }
    points {
      pos {
       x: 2.0
       y: 2.0
      }
      s: 1.0
      t: 0.1
      a: 0.0
      v: 1.0
    }
    points {
      pos {
       x: 2.0
       y: 3.0
      }
      s: 2.0
      t: 0.2
      a: 0.0
      v: 0.0
    }
    rot_rad: 0.5
    channel_probs: 0.3
    channel_probs: 0.4
    channel_probs: 0.3
    predicted_channel: 1
    cur_channel: 0
    last_confident_index:2
  )";
  google::protobuf::TextFormat::ParseFromString(text_proto, &proto);
  return proto;
}

TEST(PredictedTrajectory, Constructor) {
  const PredictedTrajectory pred(SamplePredictedTrajectoryProto());
  EXPECT_EQ(pred.probability(), 0.6);
  EXPECT_EQ(pred.annotation(), "something");
  EXPECT_EQ(pred.type(), PT_ACTNET);
  EXPECT_EQ(pred.index(), 0);
  EXPECT_FALSE(pred.is_reversed());
  EXPECT_EQ(pred.rot_rad(), 0.5);
  ASSERT_EQ(pred.points().size(), 3);
  EXPECT_DOUBLE_EQ(pred.points()[0].pos().x(), 1.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].pos().y(), 2.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].t(), 0.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].s(), 0.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].v(), 1.0);
  EXPECT_DOUBLE_EQ(pred.points()[2].pos().x(), 2.0);
  EXPECT_DOUBLE_EQ(pred.points()[2].pos().y(), 3.0);
  EXPECT_DOUBLE_EQ(pred.points()[2].t(), 0.2);
  EXPECT_DOUBLE_EQ(pred.points()[2].s(), 2.0);
  EXPECT_DOUBLE_EQ(pred.points()[2].v(), 0.0);
  EXPECT_EQ(pred.last_confident_index(), 2);
}
TEST(PredictedTrajectory, ConstructorWithTimeShift) {
  const PredictedTrajectory pred(/*shift_time=*/0.1,
                                 SamplePredictedTrajectoryProto());
  EXPECT_EQ(pred.probability(), 0.6);
  EXPECT_EQ(pred.annotation(), "something");
  EXPECT_EQ(pred.type(), PT_ACTNET);
  EXPECT_EQ(pred.index(), 0);
  EXPECT_FALSE(pred.is_reversed());
  EXPECT_EQ(pred.rot_rad(), 0.5);
  ASSERT_EQ(pred.points().size(), 2);

  EXPECT_DOUBLE_EQ(pred.points()[0].pos().x(), 2.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].pos().y(), 2.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].t(), 0.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].s(), 0.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].v(), 1.0);

  EXPECT_DOUBLE_EQ(pred.points()[1].pos().x(), 2.0);
  EXPECT_DOUBLE_EQ(pred.points()[1].pos().y(), 3.0);
  EXPECT_DOUBLE_EQ(pred.points()[1].t(), 0.1);
  EXPECT_DOUBLE_EQ(pred.points()[1].s(), 1.0);
  EXPECT_DOUBLE_EQ(pred.points()[1].v(), 0.0);
  EXPECT_EQ(pred.last_confident_index(), 1);

  // Should create empty predicted trajectory.
  const PredictedTrajectory pred_2(/*shift_time=*/0.7,
                                   SamplePredictedTrajectoryProto());
  EXPECT_EQ(pred_2.points().size(), 0);
  EXPECT_EQ(pred_2.last_confident_index(), -1);
}

TEST(PredictedTrajectory, ShiftByTime) {
  PredictedTrajectory pred(/*shift_time=*/0, SamplePredictedTrajectoryProto());
  // Shift by time to success.
  EXPECT_TRUE(pred.shift_by_time(0.1));
  EXPECT_DOUBLE_EQ(pred.points()[0].pos().x(), 2.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].pos().y(), 2.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].t(), 0.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].s(), 0.0);
  EXPECT_DOUBLE_EQ(pred.points()[0].v(), 1.0);

  EXPECT_DOUBLE_EQ(pred.points()[1].pos().x(), 2.0);
  EXPECT_DOUBLE_EQ(pred.points()[1].pos().y(), 3.0);
  EXPECT_DOUBLE_EQ(pred.points()[1].t(), 0.1);
  EXPECT_DOUBLE_EQ(pred.points()[1].s(), 1.0);
  EXPECT_DOUBLE_EQ(pred.points()[1].v(), 0.0);

  // Shift by time to fail.
  PredictedTrajectory pred_2(/*shift_time=*/0,
                             SamplePredictedTrajectoryProto());
  EXPECT_FALSE(pred_2.shift_by_time(0.5));
  EXPECT_EQ(pred_2.points().size(), 0);
  EXPECT_EQ(pred_2.last_confident_index(), -1);

  // Shift by zero.
  PredictedTrajectory pred_3(/*shift_time=*/0,
                             SamplePredictedTrajectoryProto());
  EXPECT_TRUE(pred_3.shift_by_time(0.0));
  EXPECT_EQ(pred_3.points().size(), 3);
  EXPECT_EQ(pred_3.last_confident_index(), 2);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
