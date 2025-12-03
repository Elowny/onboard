#include "onboard/planner/second_order_trajectory_point.h"

#include "absl/strings/str_format.h"

#include "onboard/math/geometry/util.h"

namespace qcraft {
namespace planner {

void SecondOrderTrajectoryPoint::FromProto(
    const SecondOrderTrajectoryPointProto& proto) {
  pos_ = Vec2dFromProto(proto.pos());
  s_ = proto.s();
  theta_ = proto.theta();
  kappa_ = proto.kappa();

  t_ = proto.t();
  v_ = proto.v();
  a_ = proto.a();
}

void SecondOrderTrajectoryPoint::ToProto(
    SecondOrderTrajectoryPointProto* proto) const {
  Vec2dToProto(pos_, proto->mutable_pos());
  proto->set_s(s_);
  proto->set_theta(theta_);
  proto->set_kappa(kappa_);

  proto->set_t(t_);
  proto->set_v(v_);
  proto->set_a(a_);
}

void SecondOrderTrajectoryPoint::FromProto(const TrajectoryPointProto& proto) {
  pos_ = Vec2dFromProto(proto.pos());
  s_ = proto.s();
  theta_ = proto.theta();
  kappa_ = proto.kappa();

  t_ = proto.t();
  v_ = proto.v();
  a_ = proto.a();
}

void SecondOrderTrajectoryPoint::ToProto(TrajectoryPointProto* proto) const {
  Vec2dToProto(pos_, proto->mutable_pos());
  proto->set_s(s_);
  proto->set_theta(theta_);
  proto->set_kappa(kappa_);

  proto->set_t(t_);
  proto->set_v(v_);
  proto->set_a(a_);
}

void SecondOrderTrajectoryPoint::ToProto(
    PoseTrajectoryPointProto* proto) const {
  Vec2dToProto(pos_, proto->mutable_pos());
  proto->set_theta(theta_);
  proto->set_t(t_);
}

std::string SecondOrderTrajectoryPoint::DebugString() const {
  return absl::StrFormat(
      "pos: (%.4f %.4f) s: %.4f theta: %.5f kappa: %.4f t: %.3f v: %.3f a: "
      "%.3f",
      pos().x(), pos().y(), s(), theta(), kappa(), t(), v(), a());
}

}  // namespace planner
}  // namespace qcraft
