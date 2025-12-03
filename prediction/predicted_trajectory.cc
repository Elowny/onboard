#include "onboard/prediction/predicted_trajectory.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>  // for floor
#include <iterator>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/prediction/prediction_message_compressor.h"

namespace qcraft {
namespace prediction {

PredictedTrajectoryPoint PredictedTrajectoryPoint::LerpPredictedTrajectoryPoint(
    const PredictedTrajectoryPoint& a, const PredictedTrajectoryPoint& b,
    double alpha) {
  PredictedTrajectoryPoint pt;
  const Vec2d lerped_pos = Lerp(a.pos(), b.pos(), alpha);
  pt.set_pos(lerped_pos);
  pt.set_a(Lerp(a.a(), b.a(), alpha));
  pt.set_s(Lerp(a.s(), b.s(), alpha));
  pt.set_v(Lerp(a.v(), b.v(), alpha));
  pt.set_kappa(Lerp(a.kappa(), b.kappa(), alpha));
  pt.set_t(Lerp(a.t(), b.t(), alpha));
  pt.set_theta(NormalizeAngle(LerpAngle(a.theta(), b.theta(), alpha)));
  const Vec2d lerped_cov = Lerp(a.nll_cov(), b.nll_cov(), alpha);
  pt.set_uncertainty(lerped_cov, NormalizeAngle(LerpAngle(
                                     a.nll_angle(), b.nll_angle(), alpha)));
  return pt;
}

void PredictedTrajectoryPoint::FromProto(
    const PredictedTrajectoryPointProto& proto) {
  set_pos(Vec2dFromProto(proto.pos()));
  set_s(proto.s());
  set_theta(proto.theta());
  set_kappa(proto.kappa());

  set_t(proto.t());
  set_v(proto.v());
  set_a(proto.a());
  if (proto.has_nll_cov()) {
    nll_cov_ = Vec2dFromProto(proto.nll_cov());
    nll_angle_ = proto.nll_angle();
  }
}

void PredictedTrajectoryPoint::ToProto(
    PredictedTrajectoryPointProto* proto) const {
  Vec2dToProto(pos(), proto->mutable_pos());
  proto->set_s(s());
  proto->set_theta(theta());
  proto->set_kappa(kappa());
  proto->set_t(t());
  proto->set_v(v());
  proto->set_a(a());
  Vec2dToProto(nll_cov_, proto->mutable_nll_cov());
  proto->set_nll_angle(nll_angle_);
}

Polygon2d PredictedTrajectory::CreateContourForPoint(
    const Polygon2d& obj_contour, int i) const {
  QCHECK(!points().empty());
  std::vector<Vec2d> new_contour_points = obj_contour.points();
  const PredictedTrajectoryPoint& pred_point0 = points()[0];
  const PredictedTrajectoryPoint& pred_point = points()[i];
  for (int j = 0; j < new_contour_points.size(); ++j) {
    new_contour_points[j] =
        Vec2d(new_contour_points[j] - pred_point0.pos())
            .FastRotate(pred_point.theta() - pred_point0.theta()) +
        pred_point.pos();
  }
  return Polygon2d(std::move(new_contour_points));
}

void PredictedTrajectory::PrintDebugInfo() const {
  QLOG(INFO) << absl::StrFormat(
      "\ttype = %s (%d), probability = %7.6f, annotation = "
      "%s, "
      "size = %d, "
      "is_reversed = %d",
      PredictionType_Name(type_), type_, probability_, annotation_,
      points_.size(), is_reversed_);
}

void PredictedTrajectory::FromProto(const PredictedTrajectoryProto& proto) {
  FromProto(/*shift_time=*/0.0, proto);
}

void PredictedTrajectory::FromProto(double shift_time,
                                    const PredictedTrajectoryProto& proto) {
  // The proto message is supposed to have been decompressed by
  // DecompressObjectsPredictionProto
  QCHECK_EQ(proto.compressed_points().x_size(), 0);
  probability_ = proto.probability();
  type_ = proto.type();
  rot_rad_ = proto.rot_rad();
  annotation_ = proto.annotation();
  index_ = proto.index();
  is_reversed_ = proto.is_reversed();
  points_.reserve(proto.points_size());
  av_relation_.no_relation = proto.av_relation_prob().no_relation();
  av_relation_.pass = proto.av_relation_prob().pass();
  av_relation_.yield = proto.av_relation_prob().yield();
  is_hard_braking_ = proto.is_hard_braking();
  cur_channel_ = proto.cur_channel();
  predicted_channel_ = proto.predicted_channel();
  lane_selection_traj_type_ = proto.lane_selection_traj_type();
  if (proto.points_size() > 0) {
    if (type_ == PT_STATIONARY) {
      const auto& pt = proto.points(0);
      for (int i = 0; i < kPredictionPointNum; ++i) {
        points_.emplace_back();
        points_.back().FromProto(pt);
        points_.back().set_t(i * kPredictionTimeStep);
      }
    } else {
      int i = 0;
      while (i < proto.points_size() && proto.points(i).t() < shift_time) {
        ++i;
      }
      if (i == 0) {
        for (int j = i; j < proto.points_size(); ++j) {
          auto& pt = points_.emplace_back(proto.points(j));
          pt.set_t(pt.t() - proto.points(i).t());
          pt.set_s(pt.s() - proto.points(i).s());
        }
      } else if (i == proto.points_size()) {
        // All points are before shift_time. Empty predicted trajectory.
        return;
      } else {
        // Calculate alpha. Same for all points because of the same timestep.
        const auto& prev_pt = proto.points(i - 1);
        const auto& pt = proto.points(i);
        const auto alpha = (shift_time - prev_pt.t()) / (pt.t() - prev_pt.t());
        double shift_s_offset = pt.s();
        for (int j = i; j < proto.points_size(); ++j) {
          const auto lerped_pt_proto = LerpPredictedTrajectoryPointProto(
              proto.points(j - 1), proto.points(j), alpha);
          if (j == i) {
            shift_s_offset = lerped_pt_proto.s();
          }
          auto& pt = points_.emplace_back(lerped_pt_proto);
          pt.set_t(pt.t() - shift_time);
          pt.set_s(pt.s() - shift_s_offset);
        }
      }
    }
  }
  last_confident_index_ =
      proto.last_confident_index() +
      static_cast<int>(std::floor(shift_time / kPredictionTimeStep));
  if (points_.empty()) {
    last_confident_index_ = -1;
  } else {
    last_confident_index_ = std::clamp(last_confident_index_, 0,
                                       static_cast<int>(points_.size()) - 1);
  }
}

void PredictedTrajectory::ToProto(PredictedTrajectoryProto* proto,
                                  bool compress_traj) const {
  proto->Clear();
  proto->set_probability(probability_);
  proto->set_type(type_);
  proto->set_annotation(annotation_);
  proto->set_index(index_);
  proto->set_cur_channel(cur_channel_);
  proto->set_lane_selection_traj_type(lane_selection_traj_type_);
  proto->set_predicted_channel(predicted_channel_);
  proto->set_is_reversed(is_reversed_);
  proto->set_rot_rad(rot_rad_);
  proto->mutable_channel_probs()->Add(channel_probs_.begin(),
                                      channel_probs_.end());
  proto->set_last_confident_index(last_confident_index_);
  PredictedTrajectoryProto::ObjectRelationProb relation;
  relation.set_no_relation(av_relation_.no_relation);
  relation.set_yield(av_relation_.yield);
  relation.set_pass(av_relation_.pass);
  std::vector<double> relation_probs = {av_relation_.no_relation,
                                        av_relation_.yield, av_relation_.pass};
  auto max_it = std::max_element(relation_probs.begin(), relation_probs.end());
  const int relation_idx = std::distance(relation_probs.begin(), max_it);
  *proto->mutable_av_relation_prob() = std::move(relation);
  proto->set_av_relation(ObjectRelation(relation_idx));
  proto->set_is_hard_braking(is_hard_braking_);

  // If object is stationary, only send out one point.
  // Planning module needs to reconstruct the full traj.
  if (!points_.empty()) {
    if (type_ == PT_STATIONARY) {
      points_.front().ToProto(proto->add_points());
    } else {
      for (const auto& point : points_) {
        point.ToProto(proto->add_points());
      }
    }
  }

  if (compress_traj) {
    CompressPredictedTrajectoryProtoByDownSampling(proto);
  }
}

// Shift predicted trajectory by assigned time. It may results in empty
// trajectory. Return bool to know whether it's successful.
bool PredictedTrajectory::shift_by_time(double shift_time) {
  if (points_.empty()) return false;
  if (type_ == PT_STATIONARY) {
    while (points_.size() < kPredictionPointNum) {
      // Patch stationary trajectory to desired  number of points.
      const auto& point = points_.front();
      points_.push_back(point);
      points_.back().set_t((points_.size() - 1) * kPredictionTimeStep);
    }
  } else {
    int i = 0;
    while (i < points_.size() && points_[i].t() < shift_time) {
      ++i;
    }
    if (i == points_.size()) {
      points_.clear();
      last_confident_index_ = -1;
      return false;
    }
    // If first point's relative t > shift_time, do nothing.
    if (i != 0) {
      // Compute alpha, should be same for all points.
      std::vector<PredictedTrajectoryPoint> aligned_points;
      aligned_points.reserve(points_.size() - i);
      const auto& prev_pt = points_[i - 1];
      const auto& pt = points_[i];
      const double alpha = (shift_time - prev_pt.t()) / (pt.t() - prev_pt.t());
      double shift_s_offset = pt.s();
      for (int j = i; j < points_.size(); ++j) {
        auto lerped_pt = PredictedTrajectoryPoint::LerpPredictedTrajectoryPoint(
            points_[j - 1], points_[j], alpha);
        if (j == i) {
          shift_s_offset = lerped_pt.s();
        }
        aligned_points.push_back(std::move(lerped_pt));
        auto& pt = aligned_points.back();
        pt.set_t(pt.t() - shift_time);
        pt.set_s(pt.s() - shift_s_offset);
      }
      points_ = std::move(aligned_points);
    }
  }
  last_confident_index_ =
      last_confident_index_ +
      static_cast<int>(std::floor(shift_time / kPredictionTimeStep));
  if (points_.empty()) {
    last_confident_index_ = -1;
  } else {
    last_confident_index_ = std::clamp(last_confident_index_, 0,
                                       static_cast<int>(points_.size()) - 1);
  }
  return true;
}

}  // namespace prediction
}  // namespace qcraft
