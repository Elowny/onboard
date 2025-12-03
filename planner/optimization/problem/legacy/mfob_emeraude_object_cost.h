#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_EMERAUDE_OBSTACLE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_EMERAUDE_OBSTACLE_COST_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/eigen.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class MfobEmeraudeObjectCost : public Cost<PROB> {
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

  using CostType = typename Cost<PROB>::CostType;

  struct Object {
    std::vector<Segment2d> lines;  // Lines in counterclockwise order
    std::vector<std::vector<double>> buffers;
    std::vector<double> gains;
    // Variables for filter
    Vec2d ref_x;
    Vec2d ref_tangent;
    double back_offset;
    double front_offset;
    enum class Type {
      kRac,
      kFac,
    };
    Type type = Type::kRac;
    // If object is enabled
    bool enable = false;
  };

  static constexpr double kNormalizedScale = 1.0;
  static constexpr double kEpsilon = 1e-5;

  MfobEmeraudeObjectCost(
      const VehicleGeometryParamsProto* vehicle_geometry_params,
      std::vector<Object> objects, std::vector<double> cascade_buffers,
      std::vector<std::string> sub_names, bool using_hessian_approximate,
      std::string name = "MfobEmeraudeObjectCost", double scale = 1.0,
      CostType cost_type = Cost<PROB>::CostType::GROUP_OBJECT)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        vehicle_geometry_params_(*CHECK_NOTNULL(vehicle_geometry_params)),
        objects_(std::move(objects)),
        num_objects_(objects_.size()),
        cascade_buffers_(cascade_buffers),
        sub_names_(std::move(sub_names)),
        using_hessian_approximate_(using_hessian_approximate) {
    QCHECK_EQ(cascade_buffers_.size(), sub_names_.size());
    penetrations_.resize(num_objects_);
    penetration_jacobians_.resize(num_objects_);
    penetration_hessians_.resize(num_objects_);
    for (int k = 0, n = num_objects_; k < n; ++k) {
      // Object prediction time may be shorter than ddp time horizon.
      const auto& obj_k = objects_[k];
      if (!obj_k.enable) continue;
      const auto gains_size = obj_k.gains.size();
      QCHECK_GE(sub_names_.size(), gains_size);
      // Object sanity check.
      for (int i = 0; i < obj_k.lines.size(); ++i) {
        QCHECK_EQ(obj_k.buffers[i].size(), gains_size);
        if (i != 0) {
          QCHECK_LE(std::abs(obj_k.lines[i - 1].unit_direction().dot(
                        obj_k.lines[i].unit_direction())),
                    kEpsilon);
          QCHECK_GE(obj_k.lines[i - 1].unit_direction().CrossProd(
                        obj_k.lines[i].unit_direction()),
                    0.0);
        }
      }
      if (obj_k.lines.size() == 4) {
        QCHECK_LE(
            (obj_k.lines.front().start() - obj_k.lines.back().end()).norm(),
            kEpsilon);
      }

      penetrations_[k].resize(gains_size,
                              std::numeric_limits<double>::infinity());
      penetration_jacobians_[k].resize(gains_size,
                                       PenetrationJacobianType::Zero());
      penetration_hessians_[k].resize(gains_size,
                                      PenetrationHessianType::Zero());
    }
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    DividedG res(sub_names_.size());
    horizon = std::min(horizon, num_objects_);
    for (int k = 0; k < horizon; ++k) {
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
      const auto& ob_k = objects_[k];
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          res.AddSubG(
              i, cascade_buffers_[i] * ob_k.gains[i] * Sqr(penetrations_k[i]));
        }
      }
    }
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
      res.SetIsSoft(i, sub_names_[i] == SoftNameString);
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType& /*x*/,
                                  const ControlType& /*u*/,
                                  bool using_scale) const override {
    DCHECK_GE(k, 0);
    std::vector<double> cascade_buffers(cascade_buffers_.size());
    if (using_scale) {
      cascade_buffers = cascade_buffers_;
    } else {
      std::fill(cascade_buffers.begin(), cascade_buffers.end(), 1.0);
    }
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    if (k >= num_objects_) return res;
    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    const auto& ob_k = objects_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        res.AddSubG(i, 0.5 * Cost<PROB>::scale() * cascade_buffers[i] *
                           ob_k.gains[i] * Sqr(penetrations_k[i]));
      }
    }
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return 0.0;

    double g = 0.0;
    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    const auto& ob_k = objects_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        g += 0.5 * Cost<PROB>::scale() * cascade_buffers_[i] * ob_k.gains[i] *
             Sqr(penetrations_k[i]);
      }
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return;

    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    const auto& ob_k = objects_[k];
    const auto& penetration_jacobians_k = penetration_jacobians_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        *dgdx += Cost<PROB>::scale() * cascade_buffers_[i] * ob_k.gains[i] *
                 penetrations_k[i] * penetration_jacobians_k[i];
      }
    }
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return;

    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    const auto& ob_k = objects_[k];
    const auto penetration_jacobians_k =
        absl::MakeConstSpan(penetration_jacobians_[k]);
    const auto penetration_hessians_k =
        absl::MakeConstSpan(penetration_hessians_[k]);
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        if (using_hessian_approximate_) {
          *ddgdxdx += Cost<PROB>::scale() * cascade_buffers_[i] *
                      ob_k.gains[i] *
                      (penetration_jacobians_k[i].transpose() *
                       penetration_jacobians_k[i]);
        } else {
          *ddgdxdx += Cost<PROB>::scale() * cascade_buffers_[i] *
                      ob_k.gains[i] *
                      (penetrations_k[i] * penetration_hessians_k[i] +
                       penetration_jacobians_k[i].transpose() *
                           penetration_jacobians_k[i]);
        }
      }
    }
  }
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

  static std::vector<double> ComputePointCrossProdOnLines(
      absl::Span<const Segment2d> lines, const Vec2d& pos) {
    std::vector<double> cross_prods;
    cross_prods.reserve(lines.size());
    for (const auto& line : lines)
      cross_prods.emplace_back(line.ProductOntoUnit(pos));
    return cross_prods;
  }

  static bool ComputePosPartition(absl::Span<const Segment2d> lines,
                                  const Vec2d& pos,
                                  absl::Span<const double> cross_pod,
                                  int* index) {
    bool is_in_corner_region = false;
    const double prod_start = lines[0].ProjectOntoUnit(pos);
    const double prod_end = lines.back().ProjectOntoUnit(pos);
    if (prod_start < 0.0 && cross_pod[0] < 0.0) {
      *index = 0;
      // If lines belong to a box.
      if (lines.size() == 4) is_in_corner_region = true;
    } else {
      if (prod_end > lines.back().length() && cross_pod.back() < 0.0) {
        *index = lines.size() - 1;
      } else {
        for (int i = 0; i < lines.size(); ++i) {
          const Segment2d& line = lines[i];
          const double dot = line.ProjectOntoUnit(pos);
          if (dot >= 0.0 && dot <= line.length() && cross_pod[i] <= 0) {
            *index = i;
            break;
          }
          if (dot < 0.0 && i != 0) {
            *index = i;
            is_in_corner_region = true;
            break;
          }
        }
      }
    }
    return is_in_corner_region;
  }

  // Line0 and line1 should be in counter clockwise order, r is distance between
  // x0 and line0.end().
  static double GetBufferInCornerRegion(double r, const Vec2d& x0,
                                        double buffer0, double buffer1,
                                        const Segment2d& /*line0*/,
                                        const Segment2d& line1) {
    const Vec2d& dir = -line1.unit_direction();
    Mat2d force_transformation;
    force_transformation.row(0) = dir;
    force_transformation.row(1) = dir.Perp();
    Vec2d pos_tmp =
        (x0 - line1.start()).transpose() * force_transformation.inverse();
    pos_tmp.x() = std::max(kEpsilon, pos_tmp.x());
    pos_tmp.y() = std::max(kEpsilon, pos_tmp.y());

    const double b = buffer0;
    const double a = buffer1;

    const double p_den = (a * pos_tmp.x() + b * pos_tmp.y());
    const double p_num = a * b;
    const double p = p_num / p_den;

    return p * r;
  }

  // TODO(runbing): Split derivatives computation from update and move into
  // UpdateDerivatives before using.
  void Update(const StatesType& xs, const ControlsType& /*us*/,
              int horizon) override {
    const double wheel_base = vehicle_geometry_params_.wheel_base();

    horizon = std::min(horizon, num_objects_);
    for (int k = 0; k < horizon; ++k) {
      auto& penetrations_k = penetrations_[k];
      auto& penetration_jacobians_k = penetration_jacobians_[k];
      auto& penetration_hessians_k = penetration_hessians_[k];

      for (int i = 0; i < penetrations_k.size(); ++i) {
        penetrations_k[i] = std::numeric_limits<double>::infinity();
        penetration_jacobians_k[i].template segment<3>(PROB::kStateXIndex) =
            Eigen::Matrix<double, 3, 1>::Zero();
        penetration_hessians_k[i].template block<3, 3>(PROB::kStateXIndex,
                                                       PROB::kStateXIndex) =
            Eigen::Matrix<double, 3, 3>::Zero();
      }
      const Object& ob = objects_[k];
      if (!ob.enable) continue;

      const auto x0 = PROB::GetStateMapAtStep(xs, k);
      const Vec2d pos = PROB::pos(xs, k);
      const Vec2d av_tangent =
          Vec2d::FastUnitFromAngleN12(PROB::StateGetTheta(x0));
      const Vec2d av_normal = av_tangent.Perp();
      VLOG(3) << "Step " << k;
      VLOG(4) << "x0 = " << x0.transpose()
              << " av_tangent = " << av_tangent.transpose();

      const auto lines = absl::MakeConstSpan(ob.lines);
      const auto& buffers = ob.buffers;

      switch (ob.type) {
        case Object::Type::kRac: {
          const double proj_on_ref_tangent =
              (pos - ob.ref_x).dot(ob.ref_tangent);
          if (proj_on_ref_tangent < ob.back_offset ||
              proj_on_ref_tangent > ob.front_offset)
            continue;
          // Cross product to inner or outer.
          std::vector<double> cross_prods =
              ComputePointCrossProdOnLines(lines, pos);
          const bool in_object = absl::c_all_of(
              cross_prods, [](double cross_prod) { return cross_prod >= 0.0; });
          if (in_object) {
            auto min_iter = absl::c_min_element(cross_prods);
            const int min_index = std::distance(cross_prods.begin(), min_iter);
            const Segment2d& line = lines[min_index];
            const Vec2d& obs_unit_dir = line.unit_direction();

            const double dist = -obs_unit_dir.CrossProd(pos - line.start());
            for (int i = 0; i < penetrations_k.size(); ++i) {
              penetrations_k[i] = dist - buffers[min_index][i];
              penetration_jacobians_k[i].template segment<2>(
                  PROB::kStateXIndex) = -obs_unit_dir.Perp();
            }
          } else {
            // Find location.
            int index = 0;
            const bool is_in_corner_region =
                ComputePosPartition(lines, pos, cross_prods, &index);
            const Segment2d& line = lines[index];
            const Vec2d diff_x0 = pos - line.start();
            // prod >= 0.0 && prod <= length
            if (!is_in_corner_region) {
              const Vec2d obs_unit_dir = line.unit_direction();
              const double dist = -obs_unit_dir.CrossProd(diff_x0);
              for (int i = 0; i < penetrations_k.size(); ++i) {
                penetrations_k[i] = dist - buffers[index][i];
                penetration_jacobians_k[i].template segment<2>(
                    PROB::kStateXIndex) = -obs_unit_dir.Perp();
              }
            } else {
              // prod <=0.0
              const double diff_x0_norm = diff_x0.norm();
              const double diff_x0_norm_inv = 1.0 / diff_x0_norm;
              const double cube_norm_inv_x0 = Cube(diff_x0_norm_inv);
              for (int i = 0; i < penetrations_k.size(); ++i) {
                penetrations_k[i] = diff_x0_norm;
                penetration_jacobians_k[i].template segment<2>(
                    PROB::kStateXIndex) = diff_x0 * diff_x0_norm_inv;
                penetration_hessians_k[i](PROB::kStateXIndex,
                                          PROB::kStateXIndex) =
                    diff_x0_norm_inv -
                    diff_x0.x() * diff_x0.x() * cube_norm_inv_x0;
                penetration_hessians_k[i](PROB::kStateXIndex,
                                          PROB::kStateYIndex) =
                    -diff_x0.x() * diff_x0.y() * cube_norm_inv_x0;
                penetration_hessians_k[i](PROB::kStateYIndex,
                                          PROB::kStateXIndex) =
                    penetration_hessians_k[i](0, 1);
                penetration_hessians_k[i](PROB::kStateYIndex,
                                          PROB::kStateYIndex) =
                    diff_x0_norm_inv -
                    diff_x0.y() * diff_x0.y() * cube_norm_inv_x0;

                GetRacBufferInfo(
                    diff_x0_norm, diff_x0_norm_inv, x0, buffers[index - 1][i],
                    buffers[index][i], lines[index - 1], lines[index], ob.type,
                    &penetrations_k[i], &penetration_jacobians_k[i],
                    &penetration_hessians_k[i]);
              }
            }
          }
          break;
        }
        case Object::Type::kFac: {
          // Cross product to inner or outer.
          const Vec2d x_fac = pos + av_tangent * wheel_base;
          const double proj_on_ref_tangent =
              (x_fac - ob.ref_x).dot(ob.ref_tangent);
          if (proj_on_ref_tangent < ob.back_offset ||
              proj_on_ref_tangent > ob.front_offset)
            continue;
          std::vector<double> cross_prods =
              ComputePointCrossProdOnLines(lines, x_fac);
          const bool in_object = absl::c_all_of(
              cross_prods, [](double cross_prod) { return cross_prod >= 0.0; });
          if (in_object) {
            // Find min_dist.
            auto min_iter = absl::c_min_element(cross_prods);
            const int min_index = std::distance(cross_prods.begin(), min_iter);
            const Segment2d& line = lines[min_index];
            const Vec2d& obs_unit_dir_perp = line.unit_direction().Perp();
            const double dist = -(x_fac - line.start()).dot(obs_unit_dir_perp);
            for (int i = 0; i < penetrations_k.size(); ++i) {
              penetrations_k[i] = dist - buffers[min_index][i];
              penetration_jacobians_k[i].template segment<2>(
                  PROB::kStateXIndex) = -obs_unit_dir_perp;
              penetration_jacobians_k[i](PROB::kStateThetaIndex) =
                  -obs_unit_dir_perp.dot(av_normal * wheel_base);
              penetration_hessians_k[i](PROB::kStateThetaIndex,
                                        PROB::kStateThetaIndex) =
                  -obs_unit_dir_perp.dot(-av_tangent * wheel_base);
            }
          } else {
            // Find location.
            int index = 0;
            const bool is_in_corner_region =
                ComputePosPartition(lines, x_fac, cross_prods, &index);
            const Segment2d& line = lines[index];
            // prod >= 0.0 && prod <= length
            if (!is_in_corner_region) {
              const Vec2d obs_unit_dir_perp = line.unit_direction().Perp();
              const double dist =
                  -(x_fac - line.start()).dot(obs_unit_dir_perp);
              for (int i = 0; i < penetrations_k.size(); ++i) {
                penetrations_k[i] = dist - buffers[index][i];
                penetration_jacobians_k[i].template segment<2>(
                    PROB::kStateXIndex) = -obs_unit_dir_perp;
                penetration_jacobians_k[i](PROB::kStateThetaIndex) =
                    -obs_unit_dir_perp.dot(av_normal * wheel_base);
                penetration_hessians_k[i](PROB::kStateThetaIndex,
                                          PROB::kStateThetaIndex) =
                    -obs_unit_dir_perp.dot(-av_tangent * wheel_base);
              }
            } else {
              // prod <=0.0
              const Vec2d diff_x1_fac = x_fac - line.start();
              const double diff_x1_fac_norm = diff_x1_fac.norm();
              const double diff_x1_fac_norm_inv = 1.0 / diff_x1_fac_norm;
              const double cube_norm_inv_x1 = Cube(diff_x1_fac_norm_inv);
              for (int i = 0; i < penetrations_k.size(); ++i) {
                penetrations_k[i] = diff_x1_fac_norm;
                penetration_jacobians_k[i].template segment<2>(
                    PROB::kStateXIndex) = diff_x1_fac * diff_x1_fac_norm_inv;
                penetration_jacobians_k[i](PROB::kStateThetaIndex) =
                    diff_x1_fac.dot(av_normal * wheel_base) *
                    diff_x1_fac_norm_inv;
                penetration_hessians_k[i](PROB::kStateXIndex,
                                          PROB::kStateXIndex) =
                    diff_x1_fac_norm_inv -
                    diff_x1_fac.x() * diff_x1_fac.x() * cube_norm_inv_x1;
                penetration_hessians_k[i](PROB::kStateXIndex,
                                          PROB::kStateYIndex) =
                    -diff_x1_fac.x() * diff_x1_fac.y() * cube_norm_inv_x1;
                penetration_hessians_k[i](PROB::kStateYIndex,
                                          PROB::kStateXIndex) =
                    penetration_hessians_k[i](PROB::kStateXIndex,
                                              PROB::kStateYIndex);
                penetration_hessians_k[i](PROB::kStateYIndex,
                                          PROB::kStateYIndex) =
                    diff_x1_fac_norm_inv -
                    diff_x1_fac.y() * diff_x1_fac.y() * cube_norm_inv_x1;
                penetration_hessians_k[i](PROB::kStateXIndex,
                                          PROB::kStateThetaIndex) =
                    av_normal.x() * wheel_base * diff_x1_fac_norm_inv -
                    cube_norm_inv_x1 * diff_x1_fac.x() *
                        diff_x1_fac.dot(av_normal * wheel_base);
                penetration_hessians_k[i](PROB::kStateYIndex,
                                          PROB::kStateThetaIndex) =
                    av_normal.y() * wheel_base * diff_x1_fac_norm_inv -
                    cube_norm_inv_x1 * diff_x1_fac.y() *
                        diff_x1_fac.dot(av_normal * wheel_base);
                penetration_hessians_k[i](PROB::kStateThetaIndex,
                                          PROB::kStateXIndex) =
                    penetration_hessians_k[i](PROB::kStateXIndex,
                                              PROB::kStateThetaIndex);
                penetration_hessians_k[i](PROB::kStateThetaIndex,
                                          PROB::kStateYIndex) =
                    penetration_hessians_k[i](PROB::kStateYIndex,
                                              PROB::kStateThetaIndex);
                penetration_hessians_k[i](PROB::kStateThetaIndex,
                                          PROB::kStateThetaIndex) =
                    (diff_x1_fac.dot(-av_tangent * wheel_base) +
                     wheel_base * av_normal.dot(av_normal * wheel_base)) *
                        diff_x1_fac_norm_inv -
                    cube_norm_inv_x1 * diff_x1_fac.dot(av_normal * wheel_base) *
                        diff_x1_fac.dot(av_normal * wheel_base);

                GetFacBufferInfo(
                    diff_x1_fac_norm, diff_x1_fac_norm_inv, x_fac, wheel_base,
                    av_tangent, buffers[index - 1][i], buffers[index][i],
                    lines[index - 1], lines[index], ob.type, &penetrations_k[i],
                    &penetration_jacobians_k[i], &penetration_hessians_k[i]);
              }
            }
          }
          break;
        }
      }
    }
  }

 protected:
  using PenetrationJacobianType = Eigen::Matrix<double, 1, PROB::kStateSize>;
  using PenetrationHessianType =
      Eigen::Matrix<double, PROB::kStateSize, PROB::kStateSize>;

  // buffer = ab / (ax + by) * sqrt(x^2 + y^2);
  // p = ab / (ax + by)
  void GetRacBufferInfo(double r, double r_inv, const StateType& x0,
                        double buffer0, double buffer1,
                        const Segment2d& /*line0*/, const Segment2d& line1,
                        typename Object::Type /*type*/, double* penetrations,
                        PenetrationJacobianType* penetration_jacobian,
                        PenetrationHessianType* penetration_hessians) {
    const Vec2d& dir = -line1.unit_direction();
    Mat2d force_transformation;
    force_transformation.row(0) = dir;
    force_transformation.row(1) = dir.Perp();
    Vec2d pos_tmp = (PROB::StateGetPos(x0) - line1.start()).transpose() *
                    force_transformation.inverse();
    pos_tmp.x() = std::max(kEpsilon, pos_tmp.x());
    pos_tmp.y() = std::max(kEpsilon, pos_tmp.y());

    const double b = buffer0;
    const double a = buffer1;

    // d(pos_tmp_x + pos_tmp_y) / dx
    const double dxsum_dx = pos_tmp.dot(dir);
    // d(pos_tmp_x + pos_tmp_y) / dy
    const double dxsum_dy = dir.CrossProd(pos_tmp);

    const double p_den = (a * pos_tmp.x() + b * pos_tmp.y());
    const double p_num = a * b;
    const double p = p_num / p_den;

    (*penetrations) -= p * r;

    const double dp = -p_num / Sqr(p_den);
    const double ddp = 2.0 * p_num / Cube(p_den);

    const double dp_den_dx = a * dir.x() + b * dir.y();
    const double dp_den_dy = -a * dir.y() + b * dir.x();

    const double dp_dx = dp * dp_den_dx;
    const double dp_dy = dp * dp_den_dy;

    const double ddp_dxdx = ddp * Sqr(dp_den_dx);
    const double ddp_dydx = ddp * dp_den_dx * dp_den_dy;
    const double ddp_dydy = ddp * Sqr(dp_den_dy);

    const double dr_dx = r_inv * dxsum_dx;
    const double dr_dy = r_inv * dxsum_dy;

    const double square_dir_dist_inv = r_inv * dir.squaredNorm();

    const double negative_cube_dist_inv = -1.0 / Cube(r);
    const double ddr_dxdx =
        negative_cube_dist_inv * Sqr(dxsum_dx) + square_dir_dist_inv;
    const double ddr_dydy =
        negative_cube_dist_inv * Sqr(dxsum_dy) + square_dir_dist_inv;
    const double ddr_dxdy = negative_cube_dist_inv * dxsum_dx * dxsum_dy;

    const double hessian_xy =
        dr_dy * dp_dx + r * ddp_dydx + dp_dy * dr_dx + p * ddr_dxdy;

    (*penetration_jacobian)(PROB::kStateXIndex) -= r * dp_dx + p * dr_dx;
    (*penetration_jacobian)(PROB::kStateYIndex) -= r * dp_dy + p * dr_dy;
    (*penetration_hessians)(PROB::kStateXIndex, PROB::kStateXIndex) -=
        dr_dx * dp_dx + r * ddp_dxdx + dp_dx * dr_dx + p * ddr_dxdx;
    (*penetration_hessians)(PROB::kStateXIndex, PROB::kStateYIndex) -=
        hessian_xy;
    (*penetration_hessians)(PROB::kStateYIndex, PROB::kStateXIndex) -=
        hessian_xy;
    (*penetration_hessians)(PROB::kStateYIndex, PROB::kStateYIndex) -=
        dr_dy * dp_dy + r * ddp_dydy + dp_dy * dr_dy + p * ddr_dydy;
  }

  void GetFacBufferInfo(double r, double r_inv, const Vec2d& x_fac,
                        double wheel_base, const Vec2d& tangent, double buffer0,
                        double buffer1, const Segment2d& /*line0*/,
                        const Segment2d& line1, typename Object::Type /*type*/,
                        double* penetrations,
                        PenetrationJacobianType* penetration_jacobian,
                        PenetrationHessianType* penetration_hessians) {
    // line1 x line2 y
    const Vec2d& dir = -line1.unit_direction();
    const Vec2d dir_perp = dir.Perp();

    Mat2d force_transformation;
    force_transformation.row(0) = dir;
    force_transformation.row(1) = dir_perp;
    Vec2d pos_tmp =
        (x_fac - line1.start()).transpose() * force_transformation.inverse();
    pos_tmp.x() = std::max(kEpsilon, pos_tmp.x());
    pos_tmp.y() = std::max(kEpsilon, pos_tmp.y());

    const double b = buffer0;
    const double a = buffer1;

    const double p_den = (a * pos_tmp.x() + b * pos_tmp.y());
    const double p_num = a * b;
    const double p = p_num / p_den;

    (*penetrations) -= p * r;

    const Vec2d& ldir = wheel_base * dir;
    const double dpos_tmp_x_dtheta = tangent.CrossProd(ldir);
    const double dpos_tmp_y_dtheta = ldir.CrossProd(tangent.Perp());
    const double dpos_tmp_x_dthetadtheta = ldir.dot(-tangent);
    const double dpos_tmp_y_dthetadtheta = ldir.CrossProd(-tangent);

    const double dp = -p_num / Sqr(p_den);
    const double ddp = 2.0 * p_num / Cube(p_den);

    const double dp_den_dx = a * dir.x() + b * dir.y();
    const double dp_den_dy = -a * dir.y() + b * dir.x();
    const double dp_den_dtheta = a * dpos_tmp_x_dtheta + b * dpos_tmp_y_dtheta;

    const double dp_dx = dp * dp_den_dx;
    const double dp_dy = dp * dp_den_dy;
    const double dp_dtheta = dp * dp_den_dtheta;

    const double ddp_dxdx = ddp * Sqr(dp_den_dx);
    const double ddp_dydy = ddp * Sqr(dp_den_dy);
    const double ddp_dthetadtheta =
        ddp * Sqr(dp_den_dtheta) +
        dp * (a * dpos_tmp_x_dthetadtheta + b * dpos_tmp_y_dthetadtheta);
    const double dp_dydx = ddp * dp_den_dx * dp_den_dy;
    const double dp_dydtheta = ddp * dp_den_dy * dp_den_dtheta;
    const double dp_dxdtheta = ddp * dp_den_dx * dp_den_dtheta;

    // d(pos_tmp_x + pos_tmp_y) / dx
    const double dxsum_dx = pos_tmp.dot(dir);
    // d(pos_tmp_x + pos_tmp_y) / dy
    const double dxsum_dy = dir.CrossProd(pos_tmp);
    // d(pos_tmp_x + pos_tmp_y) / dtheta
    const double dxsum_dtheta =
        pos_tmp.x() * dpos_tmp_x_dtheta + pos_tmp.y() * dpos_tmp_y_dtheta;

    const double dr_dx = r_inv * dxsum_dx;
    const double dr_dy = r_inv * dxsum_dy;
    const double dr_dtheta = r_inv * dxsum_dtheta;

    const double square_dir_dist_inv = r_inv * dir.squaredNorm();

    const double negative_cube_dist_inv = -1.0 / Cube(r);
    const double ddr_dxdx =
        negative_cube_dist_inv * Sqr(dxsum_dx) + square_dir_dist_inv;
    const double ddr_dydy =
        negative_cube_dist_inv * Sqr(dxsum_dy) + square_dir_dist_inv;
    const double ddr_dxdy = negative_cube_dist_inv * dxsum_dx * (dxsum_dy);

    const double ddr_dxdtheta =
        negative_cube_dist_inv * dxsum_dx * dxsum_dtheta +
        r_inv * (dir.x() * dpos_tmp_x_dtheta + dir.y() * dpos_tmp_y_dtheta);

    const double ddr_dydtheta =
        negative_cube_dist_inv * (dxsum_dy)*dxsum_dtheta +
        r_inv * (-dir.y() * dpos_tmp_x_dtheta + dir.x() * dpos_tmp_y_dtheta);

    const double ddr_dthetadtheta =
        negative_cube_dist_inv * Sqr(dxsum_dtheta) +
        1.0 / r *
            (Sqr(dpos_tmp_x_dtheta) + pos_tmp.x() * dpos_tmp_x_dthetadtheta +
             Sqr(dpos_tmp_y_dtheta) + pos_tmp.y() * dpos_tmp_y_dthetadtheta);

    (*penetration_jacobian)(PROB::kStateXIndex) -= r * dp_dx + p * dr_dx;
    (*penetration_jacobian)(1) -= r * dp_dy + p * dr_dy;
    (*penetration_jacobian)(2) -= r * dp_dtheta + p * dr_dtheta;

    const double hessian_xy =
        dr_dy * dp_dx + r * dp_dydx + dp_dy * dr_dx + p * ddr_dxdy;
    const double hessian_xtheta = dr_dtheta * dp_dx + r * dp_dxdtheta +
                                  dp_dtheta * dr_dx + p * ddr_dxdtheta;
    const double hessian_ytheta = dr_dtheta * dp_dy + r * dp_dydtheta +
                                  dp_dtheta * dr_dy + p * ddr_dydtheta;

    (*penetration_hessians)(PROB::kStateXIndex, PROB::kStateYIndex) -=
        hessian_xy;
    (*penetration_hessians)(PROB::kStateYIndex, PROB::kStateXIndex) -=
        hessian_xy;
    (*penetration_hessians)(PROB::kStateXIndex, PROB::kStateThetaIndex) -=
        hessian_xtheta;
    (*penetration_hessians)(PROB::kStateThetaIndex, PROB::kStateXIndex) -=
        hessian_xtheta;
    (*penetration_hessians)(PROB::kStateYIndex, PROB::kStateThetaIndex) -=
        hessian_ytheta;
    (*penetration_hessians)(PROB::kStateThetaIndex, PROB::kStateYIndex) -=
        hessian_ytheta;

    (*penetration_hessians)(PROB::kStateXIndex, PROB::kStateXIndex) -=
        dr_dx * dp_dx + r * ddp_dxdx + dp_dx * dr_dx + p * ddr_dxdx;
    (*penetration_hessians)(PROB::kStateYIndex, PROB::kStateYIndex) -=
        dr_dy * dp_dy + r * ddp_dydy + dp_dy * dr_dy + p * ddr_dydy;
    (*penetration_hessians)(PROB::kStateThetaIndex, PROB::kStateThetaIndex) -=
        dr_dtheta * dp_dtheta + r * ddp_dthetadtheta + dp_dtheta * dr_dtheta +
        p * ddr_dthetadtheta;
  }

 private:
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::vector<Object> objects_;
  int num_objects_;
  std::vector<double> cascade_buffers_;
  std::vector<std::string> sub_names_;
  bool using_hessian_approximate_;

  // States.
  std::vector<std::vector<double>> penetrations_;
  std::vector<std::vector<PenetrationJacobianType>> penetration_jacobians_;
  std::vector<std::vector<PenetrationHessianType>> penetration_hessians_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_EMERAUDE_OBSTACLE_COST_H_
