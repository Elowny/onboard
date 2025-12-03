#ifndef ONBOARD_PREDICTION_PREDICTED_TRAJECTORY_H_
#define ONBOARD_PREDICTION_PREDICTED_TRAJECTORY_H_

#include <algorithm>
#include <iterator>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
class alignas(64) PredictedTrajectoryPoint
    : public planner::SecondOrderTrajectoryPoint {
 public:
  PredictedTrajectoryPoint() = default;
  explicit PredictedTrajectoryPoint(
      const PredictedTrajectoryPointProto& proto) {
    FromProto(proto);
  }
  // Create from a second-order trajectory point where bmv info is lost.
  explicit PredictedTrajectoryPoint(
      const planner::SecondOrderTrajectoryPoint& point)
      : planner::SecondOrderTrajectoryPoint(point) {}

  inline const Vec2d& nll_cov() const { return nll_cov_; }
  inline double nll_angle() const { return nll_angle_; }

  static PredictedTrajectoryPoint LerpPredictedTrajectoryPoint(
      const PredictedTrajectoryPoint& a, const PredictedTrajectoryPoint& b,
      double alpha);

  void set_uncertainty(const Vec2d& cov, double nll_angle) {
    nll_cov_.x() = cov.x();
    nll_cov_.y() = cov.y();

    // Angle between main axis coord and current smooth coord.
    nll_angle_ = nll_angle;
  }

  // Serialization to proto.
  void FromProto(const PredictedTrajectoryPointProto& proto);
  void ToProto(PredictedTrajectoryPointProto* proto) const;

 private:
  Vec2d nll_cov_ = {0.0, 0.0};
  double nll_angle_ = 0.0;
};

class PredictedTrajectory {
 public:
  PredictedTrajectory() {}
  explicit PredictedTrajectory(const PredictedTrajectoryProto& proto) {
    FromProto(/*shift_time=*/0.0, proto);
  }
  // Build trajectory with a given shift time.
  // TODO(lidong): Change it to a builder function.
  explicit PredictedTrajectory(double shift_time,
                               const PredictedTrajectoryProto& proto) {
    FromProto(shift_time, proto);
  }
  PredictedTrajectory(double probability, std::string annotation,
                      PredictionType type, int index,
                      std::vector<PredictedTrajectoryPoint> points,
                      bool is_reversed, bool is_hard_braking = false)
      : probability_(probability),
        annotation_(std::move(annotation)),
        type_(type),
        index_(index),
        points_(std::move(points)),
        is_reversed_(is_reversed),
        is_hard_braking_(is_hard_braking),
        predicted_channel_(-1),
        cur_channel_(-1),
        lane_selection_traj_type_(PredictedTrajectoryProto::LSTT_VOID) {
    av_relation_ =
        AVObjectRelation{.no_relation = 0.0, .yield = 0.0, .pass = 0.0};
    last_confident_index_ = static_cast<int>(points_.size()) - 1;
  }

  PredictedTrajectory(double probability, std::string annotation,
                      PredictionType type, int index,
                      std::vector<PredictedTrajectoryPoint> points,
                      bool is_reversed, int last_confident_index,
                      bool is_hard_braking = false)
      : probability_(probability),
        annotation_(std::move(annotation)),
        type_(type),
        index_(index),
        points_(std::move(points)),
        is_reversed_(is_reversed),
        last_confident_index_(last_confident_index),
        is_hard_braking_(is_hard_braking),
        predicted_channel_(-1),
        cur_channel_(-1),
        lane_selection_traj_type_(PredictedTrajectoryProto::LSTT_VOID) {
    last_confident_index_ =
        std::min(last_confident_index_, static_cast<int>(points_.size()) - 1);
    av_relation_ =
        AVObjectRelation{.no_relation = 0.0, .yield = 0.0, .pass = 0.0};
  }
  PredictedTrajectory(double probability, std::string annotation,
                      PredictionType type, int index,
                      std::vector<PredictedTrajectoryPoint> points,
                      bool is_reversed, AVObjectRelation relation,
                      bool is_hard_braking = false)
      : probability_(probability),
        annotation_(std::move(annotation)),
        type_(type),
        index_(index),
        points_(std::move(points)),
        is_reversed_(is_reversed),
        av_relation_(relation),
        is_hard_braking_(is_hard_braking),
        predicted_channel_(-1),
        cur_channel_(-1),
        lane_selection_traj_type_(PredictedTrajectoryProto::LSTT_VOID) {
    last_confident_index_ = static_cast<int>(points_.size()) - 1;
  }

  double probability() const { return probability_; }
  int predicted_channel() const { return predicted_channel_; }
  const std::string& annotation() const { return annotation_; }
  PredictionType type() const { return type_; }
  bool is_reversed() const { return is_reversed_; }
  int index() const { return index_; }
  int last_confident_index() const { return last_confident_index_; }
  const std::vector<PredictedTrajectoryPoint>& points() const {
    return points_;
  }
  double rot_rad() const { return rot_rad_; }
  bool is_hard_braking() const { return is_hard_braking_; }

  void set_rot_rad(double rot_rad) { rot_rad_ = rot_rad; }
  void set_probability(double probability) { probability_ = probability; }
  void set_annotation(std::string annotation) {
    annotation_ = std::move(annotation);
  }
  void set_type(PredictionType type) { type_ = type; }
  void set_index(int index) { index_ = index; }

  void set_relation_probability(absl::Span<const double> relation_probs) {
    av_relation_.no_relation = relation_probs[0];
    av_relation_.yield = relation_probs[1];
    av_relation_.pass = relation_probs[2];
  }
  void set_relation(AVObjectRelation obj_relation) {
    av_relation_ = obj_relation;
  }
  void set_predicted_channel(int predicted_channel) {
    predicted_channel_ = predicted_channel;
  }
  void set_cur_channel(int cur_channel) { cur_channel_ = cur_channel; }
  void set_lane_selection_traj_type(
      PredictedTrajectoryProto::LaneSelectionTrajType
          lane_selection_traj_type) {
    lane_selection_traj_type_ = lane_selection_traj_type;
  }
  void set_channel_probs(absl::Span<const double> channel_probs) {
    std::copy(channel_probs.begin(), channel_probs.end(),
              std::back_inserter(channel_probs_));
  }

  bool shift_by_time(double shift_time);
  std::vector<PredictedTrajectoryPoint>* mutable_points() { return &points_; }

  Polygon2d CreateContourForPoint(const Polygon2d& obj_contour, int i) const;

  void PrintDebugInfo() const;

  // Serialization to proto.
  // Can only read proto from decompressed message.
  void FromProto(const PredictedTrajectoryProto& proto);
  void ToProto(PredictedTrajectoryProto* proto, bool compress_traj) const;

 private:
  // Construct from proto with a relative timestamp.
  void FromProto(double shift_time, const PredictedTrajectoryProto& proto);

  double probability_ = 0.0;
  std::string annotation_;
  PredictionType type_;
  int index_;
  std::vector<PredictedTrajectoryPoint> points_;

  bool is_reversed_ = false;
  AVObjectRelation av_relation_;
  int last_confident_index_ = -1;
  bool is_hard_braking_ = false;
  std::vector<double> channel_probs_;
  int predicted_channel_ = -1;
  int cur_channel_ = -1;
  PredictedTrajectoryProto::LaneSelectionTrajType lane_selection_traj_type_ =
      PredictedTrajectoryProto::LSTT_VOID;
  double rot_rad_;  // Rotation angle from agent coord to smooth coord.
};

using ObjectsPredTrajsMap =
    std::map<std::string, std::vector<PredictedTrajectory>>;

struct ObjectActNetPred {
  std::vector<PredictedTrajectory> pred_trajs;
  std::optional<double> startup_prob;
};
using ObjectsActNetPredMap = std::map<ObjectIDType, ObjectActNetPred>;

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTED_TRAJECTORY_H_
