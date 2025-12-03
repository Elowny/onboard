#include "onboard/prediction/prediction_message_compressor.h"

#include <algorithm>
#include <ostream>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {

namespace {
constexpr int kDownSampleRate = 2;

void DecompressDownSampledPredictedTrajectoryProto(
    PredictedTrajectoryProto* traj) {
  if (!traj->has_compressed_points()) {
    return;
  }
  if (traj->type() == PT_STATIONARY) {
    traj->mutable_compressed_points()->Clear();
    return;
  }
  std::vector<PredictedTrajectoryPointProto> points;
  std::vector<double> ts;
  const auto& fpt = traj->compressed_points();
  points.reserve(fpt.x_size());
  ts.reserve(fpt.x_size());
  for (int i = 0; i < fpt.x_size(); ++i) {
    auto& pt = points.emplace_back();
    Vec2d(fpt.x(i), fpt.y(i)).ToProto(pt.mutable_pos());
    pt.set_s(fpt.s(i));
    pt.set_theta(fpt.theta(i));
    pt.set_kappa(fpt.kappa(i));
    pt.set_t(fpt.t(i));
    pt.set_v(fpt.v(i));
    pt.set_a(fpt.a(i));
    Vec2d(fpt.nll_cov_x(i), fpt.nll_cov_y(i)).ToProto(pt.mutable_nll_cov());
    pt.set_nll_angle(fpt.nll_angle(i));
    ts.push_back(fpt.t(i));
  }
  if (points.size() <= 1) {
    *traj->mutable_points() = {points.begin(), points.end()};
    traj->mutable_compressed_points()->Clear();
    return;
  }
  const double front_t = points.front().t();
  const PiecewiseLinearFunction<PredictedTrajectoryPointProto, double,
                                PredictedTrajectoryPointProtoLerper>
      point_proto_plf(ts, points);
  const double duration = points.back().t() - points.front().t();
  const int num_points = static_cast<int>(duration / kPredictionTimeStep) + 1;
  traj->mutable_points()->Reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    const double cur_t = front_t + i * kPredictionTimeStep;
    traj->mutable_points()->Add(point_proto_plf(cur_t));
  }
  traj->mutable_compressed_points()->Clear();
}
}  // namespace

PredictedTrajectoryPointProto LerpPredictedTrajectoryPointProto(
    const PredictedTrajectoryPointProto& a,
    const PredictedTrajectoryPointProto& b, double alpha) {
  PredictedTrajectoryPointProto pt;
  const Vec2d lerped_pos =
      Lerp(Vec2dFromProto(a.pos()), Vec2dFromProto(b.pos()), alpha);
  lerped_pos.ToProto(pt.mutable_pos());
  pt.set_s(Lerp(a.s(), b.s(), alpha));
  pt.set_theta(NormalizeAngle(LerpAngle(a.theta(), b.theta(), alpha)));
  pt.set_kappa(Lerp(a.kappa(), b.kappa(), alpha));
  pt.set_t(Lerp(a.t(), b.t(), alpha));
  pt.set_v(Lerp(a.v(), b.v(), alpha));
  pt.set_a(Lerp(a.a(), b.a(), alpha));
  const Vec2d lerped_cov =
      Lerp(Vec2dFromProto(a.nll_cov()), Vec2dFromProto(b.nll_cov()), alpha);
  lerped_cov.ToProto(pt.mutable_nll_cov());
  pt.set_nll_angle(
      NormalizeAngle(LerpAngle(a.nll_angle(), b.nll_angle(), alpha)));

  return pt;
}

void CompressPredictedTrajectoryProtoByDownSampling(
    PredictedTrajectoryProto* traj) {
  if (traj->type() == PT_STATIONARY) {
    return;
  }
  const int num_traj_pts = traj->points_size();
  if (num_traj_pts == 0) {
    return;
  }
  const auto& pts = *traj->mutable_points();
  auto& fpt = *traj->mutable_compressed_points();
  for (int i = 0; i < num_traj_pts; ++i) {
    if (i % kDownSampleRate == 0 || i == num_traj_pts - 1) {
      fpt.add_x(pts[i].pos().x());
      fpt.add_y(pts[i].pos().y());
      fpt.add_s(pts[i].s());
      fpt.add_theta(pts[i].theta());
      fpt.add_kappa(pts[i].kappa());
      fpt.add_t(pts[i].t());
      fpt.add_v(pts[i].v());
      fpt.add_a(pts[i].a());
      fpt.add_nll_cov_x(pts[i].nll_cov().x());
      fpt.add_nll_cov_y(pts[i].nll_cov().y());
      fpt.add_nll_angle(pts[i].nll_angle());
    }
  }
  traj->clear_points();
}

void DecompressPredictedTrajectoryProto(PredictedTrajectoryProto* traj) {
  // Normal procedure.
  if (traj->has_compressed_points()) {
    DecompressDownSampledPredictedTrajectoryProto(traj);
  }
}

void DecompressObjectsPredictionProto(ObjectsPredictionProto* proto) {
  FUNC_QTRACE();
  for (auto& obj : *proto->mutable_objects()) {
    for (auto& traj : *obj.mutable_trajectories()) {
      DecompressPredictedTrajectoryProto(&traj);
    }
    for (auto& traj : *obj.mutable_startup_trajs()) {
      DecompressPredictedTrajectoryProto(&traj);
    }
  }
}

}  // namespace prediction
}  // namespace qcraft
