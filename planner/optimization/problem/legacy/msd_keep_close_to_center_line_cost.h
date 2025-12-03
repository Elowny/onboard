#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MSD_KEEP_CLOSE_TO_CENTER_LINE_COST_H_  // NOLINT
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MSD_KEEP_CLOSE_TO_CENTER_LINE_COST_H_  // NOLINT

#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/util/min_segment_distance_problem.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// Use 3 circles to represent ADV
// Will limit ADV inside the region defined by center line MSD.
template <typename PROB>
class MsdKeepCloseToCenterLineCost : public Cost<PROB> {
 public:
  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  using GType = typename PROB::GType;
  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;
  using DDGDxDxType = typename PROB::DDGDxDxType;
  using DDGDxDuType = typename PROB::DDGDxDuType;
  using DDGDuDxType = typename PROB::DDGDuDxType;
  using DDGDuDuType = typename PROB::DDGDuDuType;

  using DividedG = typename Cost<PROB>::DividedG;

  static constexpr double kNormalizedScale = 10.0;
  MsdKeepCloseToCenterLineCost(
      int horizon, VehicleGeometryParamsProto vehicle_geometry_params,
      const MinSegmentDistanceProblem& center_line_msd,
      double max_distance_away_from_center_line = 10.0,
      const std::vector<double>& cascade_buffers = {0.0},
      const std::vector<double>& cascade_gains = {1.0},
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "MsdKeepCloseToCenterLineCost"),
      double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale),
        horizon_(horizon),
        vehicle_geometry_params_(std::move(vehicle_geometry_params)),
        center_line_msd_(center_line_msd),
        max_distance_away_from_center_line_(
            max_distance_away_from_center_line) {
    QCHECK_GT(horizon_, 0);
    QCHECK_GE(max_distance_away_from_center_line_, 0.0);

    const double wheel_base = vehicle_geometry_params_.wheel_base();
    const double half_vehicle_width = 0.5 * vehicle_geometry_params_.width();

    circus_center_offsets_ =
        std::vector<double>{0.0, 0.5 * wheel_base, wheel_base};

    int n = cascade_buffers.size();
    QCHECK_GT(n, 0);
    QCHECK_EQ(n, cascade_gains.size());

    cascade_radius_.reserve(n);
    cascade_gains_final_.reserve(n);

    for (int i = 0; i < n; ++i) {
      cascade_radius_.push_back(cascade_buffers[i] + half_vehicle_width);
      cascade_gains_final_.push_back(cascade_gains[i] * Cost<PROB>::scale() *
                                     0.5);
    }

    updated_results_.resize(horizon_);
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    DCHECK_EQ(horizon_, horizon);
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon_; ++k) {
      res.AddSubG(/*idx=*/0, updated_results_[k].g);
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    return updated_results_[k].g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    *dgdx = updated_results_[k].dg_dx;
  }

  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    *ddgdxdx = updated_results_[k].d2g_dx2;
  }

  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

  // TODO(runbing): Split derivatives computation from update and move into
  // UpdateDerivatives before using.
  void Update(const StatesType& xs, const ControlsType& /*us*/,
              int horizon) override {
    DCHECK_EQ(horizon_, horizon);
    for (int k = 0; k < horizon_; ++k) {
      const StateType& state = PROB::GetStateAtStep(xs, k);
      updated_results_[k] = UpdateOneStepWithDerivative(state);
    }
  }

 private:
  struct OneStepResult {
    double g = 0.0;
    DGDxType dg_dx;
    DGDuType dg_du;
    DDGDxDxType d2g_dx2;
    DDGDuDuType d2g_du2;
    DDGDuDxType d2g_du_dx;

    void SetZero() {
      g = 0.0;
      dg_dx.setZero();
      dg_du.setZero();
      d2g_dx2.setZero();
      d2g_du2.setZero();
      d2g_du_dx.setZero();
    }
  };

  OneStepResult UpdateOneStepWithDerivative(const StateType& state) {
    const double x = state[PROB::kStateXIndex];
    const double y = state[PROB::kStateYIndex];
    const double theta = state[PROB::kStateThetaIndex];
    double theta_cos_sin[2];
    fast_math::CosAndSin<12>(theta, theta_cos_sin);
    const double sin_theta = theta_cos_sin[1];
    const double cos_theta = theta_cos_sin[0];

    OneStepResult result;
    result.SetZero();

    double* p_g = &result.g;
    double* p_dg_dx = &result.dg_dx(PROB::kStateXIndex);
    double* p_dg_dy = &result.dg_dx(PROB::kStateYIndex);
    double* p_dg_dtheta = &result.dg_dx(PROB::kStateThetaIndex);

    double* p_d2g_dx2 = &result.d2g_dx2(PROB::kStateXIndex, PROB::kStateXIndex);
    double* p_d2g_dy2 = &result.d2g_dx2(PROB::kStateYIndex, PROB::kStateYIndex);
    double* p_d2g_dtheta2 =
        &result.d2g_dx2(PROB::kStateThetaIndex, PROB::kStateThetaIndex);
    double* p_d2g_dx_dy =
        &result.d2g_dx2(PROB::kStateXIndex, PROB::kStateYIndex);
    double* p_d2g_dx_dtheta =
        &result.d2g_dx2(PROB::kStateXIndex, PROB::kStateThetaIndex);
    double* p_d2g_dy_dtheta =
        &result.d2g_dx2(PROB::kStateYIndex, PROB::kStateThetaIndex);

    for (const double offset : circus_center_offsets_) {
      AppendOneCircleCostWithDerivative(
          x, y, sin_theta, cos_theta, offset, /*rightward_shift=*/0.0,
          cascade_radius_, cascade_gains_final_, p_g, p_dg_dx, p_dg_dy,
          p_dg_dtheta, p_d2g_dx2, p_d2g_dy2, p_d2g_dtheta2, p_d2g_dx_dy,
          p_d2g_dx_dtheta, p_d2g_dy_dtheta);
    }

    result.d2g_dx2(PROB::kStateYIndex, PROB::kStateXIndex) =
        result.d2g_dx2(PROB::kStateXIndex, PROB::kStateYIndex);
    result.d2g_dx2(PROB::kStateThetaIndex, PROB::kStateXIndex) =
        result.d2g_dx2(PROB::kStateXIndex, PROB::kStateThetaIndex);
    result.d2g_dx2(PROB::kStateThetaIndex, PROB::kStateYIndex) =
        result.d2g_dx2(PROB::kStateYIndex, PROB::kStateThetaIndex);
    return result;
  }

  double UpdateOneStep(const StateType& state) {
    const double x = state[PROB::kStateXIndex];
    const double y = state[PROB::kStateYIndex];
    const double theta = state[PROB::kStateThetaIndex];
    double theta_cos_sin[2];
    fast_math::CosAndSin<12>(theta, theta_cos_sin);
    const double sin_theta = theta_cos_sin[1];
    const double cos_theta = theta_cos_sin[0];

    double g = 0.0;
    for (const double offset : circus_center_offsets_) {
      AppendOneCircleCost(x, y, sin_theta, cos_theta, offset,
                          /*rightward_shift=*/0.0, cascade_radius_,
                          cascade_gains_final_, &g);
    }

    return g;
  }

  void AppendOneCircleCostWithDerivative(
      double x, double y, double sin_theta, double cos_theta,
      double forward_shift, double rightward_shift,
      const std::vector<double>& radius_list,
      const std::vector<double>& gain_list, double* pg, double* dg_dx,
      double* dg_dy, double* dg_dtheta, double* d2g_dx2, double* d2g_dy2,
      double* d2g_dtheta2, double* d2g_dx_dy, double* d2g_dx_dtheta,
      double* d2g_dy_dtheta) const {
    DCHECK_NOTNULL(pg);
    DCHECK_NOTNULL(dg_dx);
    DCHECK_NOTNULL(dg_dy);
    DCHECK_NOTNULL(dg_dtheta);
    DCHECK_NOTNULL(d2g_dx2);
    DCHECK_NOTNULL(d2g_dy2);
    DCHECK_NOTNULL(d2g_dtheta2);
    DCHECK_NOTNULL(d2g_dx_dy);
    DCHECK_NOTNULL(d2g_dx_dtheta);
    DCHECK_NOTNULL(d2g_dy_dtheta);

    MinSegmentDistanceProblem::SecondOrderDerivativeType der;
    const double xr =
        x + forward_shift * cos_theta + rightward_shift * sin_theta;
    const double yr =
        y + forward_shift * sin_theta - rightward_shift * cos_theta;
    const double min_dis = center_line_msd_.EvaluateWithSecondOrderDerivatives(
        Vec2d{xr, yr}, &der);

    for (int i = 0; i < radius_list.size(); ++i) {
      const double gain = gain_list[i];

      const double invasion =
          min_dis + radius_list[i] - max_distance_away_from_center_line_;
      if (invasion <= 0.0) {
        continue;
      }
      const double delta_y = y - yr;
      const double delta_x = x - xr;
      const double dmsd_dx = der.df_dx;
      const double dmsd_dy = der.df_dy;
      const double dmsd_dtheta = delta_y * der.df_dx - delta_x * der.df_dy;
      const double d2msd_dx_dx = der.d2f_dx_dx;
      const double d2msd_dx_dy = der.d2f_dx_dy;
      const double d2msd_dx_dtheta =
          delta_y * der.d2f_dx_dx - delta_x * der.d2f_dx_dy;
      const double d2msd_dy_dy = der.d2f_dy_dy;
      const double d2msd_dy_dtheta =
          delta_y * der.d2f_dx_dy - delta_x * der.d2f_dy_dy;
      const double d2msd_dtheta_dtheta =
          delta_y * (delta_y * der.d2f_dx_dx - delta_x * der.d2f_dx_dy) +
          delta_y * der.df_dy + delta_x * der.df_dx -
          delta_x * (delta_y * der.d2f_dx_dy - delta_x * der.d2f_dy_dy);

      const double g = gain * invasion * invasion;
      const double dg_dmsd = 2.0 * invasion * gain;

      *pg += g;
      *dg_dx += dg_dmsd * dmsd_dx;
      *dg_dy += dg_dmsd * dmsd_dy;
      *dg_dtheta += dg_dmsd * dmsd_dtheta;
      *d2g_dx2 += dg_dmsd * d2msd_dx_dx + 2.0 * gain * dmsd_dx * dmsd_dx;
      *d2g_dy2 += dg_dmsd * d2msd_dy_dy + 2.0 * gain * dmsd_dy * dmsd_dy;
      *d2g_dtheta2 += dg_dmsd * d2msd_dtheta_dtheta +
                      2.0 * gain * dmsd_dtheta * dmsd_dtheta;
      *d2g_dx_dy += dg_dmsd * d2msd_dx_dy + 2.0 * gain * dmsd_dx * dmsd_dy;
      *d2g_dx_dtheta +=
          dg_dmsd * d2msd_dx_dtheta + 2.0 * gain * dmsd_dx * dmsd_dtheta;
      *d2g_dy_dtheta +=
          dg_dmsd * d2msd_dy_dtheta + 2.0 * gain * dmsd_dy * dmsd_dtheta;
    }
  }

  void AppendOneCircleCost(double x, double y, double sin_theta,
                           double cos_theta, double forward_shift,
                           double rightward_shift,
                           const std::vector<double>& radius_list,
                           const std::vector<double>& gain_list,
                           double* pg) const {
    DCHECK_NOTNULL(pg);
    const double xr =
        x + forward_shift * cos_theta + rightward_shift * sin_theta;
    const double yr =
        y + forward_shift * sin_theta - rightward_shift * cos_theta;
    const double min_dis = center_line_msd_.Evaluate(Vec2d{xr, yr});

    for (int i = 0; i < radius_list.size(); ++i) {
      const double invasion =
          min_dis + radius_list[i] - max_distance_away_from_center_line_;
      if (invasion <= 0.0) {
        continue;
      }
      *pg += gain_list[i] * invasion * invasion;
    }
  }

  int horizon_ = 0;
  VehicleGeometryParamsProto vehicle_geometry_params_;
  MinSegmentDistanceProblem center_line_msd_;
  double max_distance_away_from_center_line_ = 0.0;

  std::vector<double> circus_center_offsets_;
  std::vector<double> cascade_radius_;
  std::vector<double> cascade_gains_final_;

  std::vector<OneStepResult> updated_results_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MSD_KEEP_CLOSE_TO_CENTER_LINE_COST_H_
