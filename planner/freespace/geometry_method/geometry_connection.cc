#include "onboard/planner/freespace/geometry_method/geometry_connection.h"

#include <cmath>
#include <vector>

#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_util.h"

namespace qcraft {
namespace planner {

bool CircleCircleConection(const GeometryMethodPoint& start,
                           const GeometryMethodPoint& goal, double max_kappa,
                           LineCirclePath* result) {
  // Convert start pose to origin.
  const Vec2d pos = goal.pos - start.pos;
  const double delta_theta = NormalizeAngle(goal.theta - start.theta);
  constexpr double kEpsilon = 0.01;
  if (std::abs(delta_theta) > kEpsilon) return false;
  GeometryMethodPoint end = {
      .pos = pos.Rotate(start.tangent.x(), -start.tangent.y()),
      .theta = delta_theta,
      .tangent = Vec2d::FastUnitFromAngle(delta_theta)};

  // Symmetric transformation.
  const bool x_symetric = (end.pos.y() < 0.0);
  const bool y_symetric = (end.pos.x() < 0.0);
  if (x_symetric) {
    end.pos = Vec2d(end.pos.x(), -end.pos.y());
    end.theta = -end.theta;
    end.tangent = Vec2d(end.tangent.x(), -end.tangent.y());
  }
  if (y_symetric) {
    end.pos = Vec2d(-end.pos.x(), end.pos.y());
    end.theta = -end.theta;
    end.tangent = Vec2d(end.tangent.x(), -end.tangent.y());
  }

  // Check if out of required region.
  constexpr double kMaxTheta = 0.2 * M_PI;
  if (std::abs(end.theta) > kMaxTheta) {
    return false;
  }

  // Connect start and end.
  const double r_min = 1.0 / max_kappa;
  const double sin_theta =
      2.0 * end.pos.x() * end.pos.y() / (Sqr(end.pos.x()) + Sqr(end.pos.y()));
  const double r = end.pos.x() / sin_theta - r_min;
  if (r < r_min) return false;
  const double theta = std::asin(sin_theta);
  const double alpha = NormalizeAngle(theta - end.theta);
  if (alpha < 0.0) return false;

  // Fill results.
  result->start = start;
  if (x_symetric) {
    result->types = {GeometryPathType::RIGHT, GeometryPathType::LEFT};
  } else {
    result->types = {GeometryPathType::LEFT, GeometryPathType::RIGHT};
  }
  if (y_symetric) {
    result->lengths = {-theta * r_min, -alpha * r};
  } else {
    result->lengths = {theta * r_min, alpha * r};
  }
  result->kappas = {max_kappa, 1.0 / r};
  const auto end1 = ExtendPathByConstantKappa(
      start, result->kappas[0], result->lengths[0], result->types[0]);
  const auto end2 = ExtendPathByConstantKappa(
      end1, result->kappas[1], result->lengths[1], result->types[1]);
  result->ends = {end1, end2};

  return true;
}

bool CircleLineConection(const GeometryMethodPoint& start,
                         const GeometryMethodPoint& goal, double max_kappa,
                         LineCirclePath* result) {
  // Convert start pose to origin.
  const Vec2d pos = goal.pos - start.pos;
  const double delta_theta = NormalizeAngle(goal.theta - start.theta);
  GeometryMethodPoint end = {
      .pos = pos.Rotate(start.tangent.x(), -start.tangent.y()),
      .theta = delta_theta,
      .tangent = Vec2d::FastUnitFromAngle(delta_theta)};

  // Symmetric transformation.
  const bool x_symetric = (end.pos.y() < 0.0);
  const bool y_symetric = (end.pos.x() < 0.0);
  if (x_symetric) {
    end.pos = Vec2d(end.pos.x(), -end.pos.y());
    end.theta = -end.theta;
    end.tangent = Vec2d(end.tangent.x(), -end.tangent.y());
  }
  if (y_symetric) {
    end.pos = Vec2d(-end.pos.x(), end.pos.y());
    end.theta = -end.theta;
    end.tangent = Vec2d(end.tangent.x(), -end.tangent.y());
  }

  // Check if out of required region.
  if (end.theta < 0.0 || end.theta > M_PI) {
    return false;
  }

  // Connect by a single line.
  constexpr double kEpsilon = 1e-6;
  if (1.0 - end.tangent.x() < kEpsilon) {
    if (end.pos.y() > kEpsilon) return false;
    result->start = start;
    result->types = {GeometryPathType::STRAIGHT};
    result->lengths = {y_symetric ? -end.pos.x() : end.pos.x()};
    result->kappas = {0.0};
    result->ends = {goal};
    return true;
  }

  // Connect start and end.
  const double r_min = 1.0 / max_kappa;
  // Firstly try line-circle connection.
  bool line_first = true;
  const double tmp = 1.0 / (1.0 - end.tangent.x());
  double r = end.pos.y() * tmp;
  double l = end.pos.x() - r * end.tangent.y();
  // If line-circle connection is illegal, try circle-line.
  if (r < r_min || l < 0.0) {
    line_first = false;
    l = end.pos.y() * end.tangent.y() * tmp - end.pos.x();
    if (l < 0.0) return false;
    r = (end.pos.y() - l * end.tangent.y()) * tmp;
    if (r < r_min) return false;
  }

  // Fill results.
  result->start = start;
  if (line_first) {
    result->types = {GeometryPathType::STRAIGHT, x_symetric
                                                     ? GeometryPathType::RIGHT
                                                     : GeometryPathType::LEFT};
    result->lengths = {l, end.theta * r};
    result->kappas = {0.0, 1.0 / r};
  } else {
    result->types = {
        x_symetric ? GeometryPathType::RIGHT : GeometryPathType::LEFT,
        GeometryPathType::STRAIGHT};
    result->lengths = {end.theta * r, l};
    result->kappas = {1.0 / r, 0.0};
  }
  if (y_symetric) {
    result->lengths = {-result->lengths[0], -result->lengths[1]};
  }
  const auto end1 = ExtendPathByConstantKappa(
      start, result->kappas[0], result->lengths[0], result->types[0]);
  const auto end2 = ExtendPathByConstantKappa(
      end1, result->kappas[1], result->lengths[1], result->types[1]);
  result->ends = {end1, end2};

  return true;
}

bool LineCircleLineConection(const GeometryMethodPoint& start,
                             const GeometryMethodPoint& goal, double max_kappa,
                             LineCirclePath* result) {
  // Convert start pose to origin.
  const Vec2d pos = goal.pos - start.pos;
  const double delta_theta = NormalizeAngle(goal.theta - start.theta);
  GeometryMethodPoint end = {
      .pos = pos.Rotate(start.tangent.x(), -start.tangent.y()),
      .theta = delta_theta,
      .tangent = Vec2d::FastUnitFromAngle(delta_theta)};

  // Symmetric transformation.
  const bool x_symetric = (end.pos.y() < 0.0);
  const bool y_symetric = (end.pos.x() < 0.0);
  if (x_symetric) {
    end.pos = Vec2d(end.pos.x(), -end.pos.y());
    end.theta = -end.theta;
    end.tangent = Vec2d(end.tangent.x(), -end.tangent.y());
  }
  if (y_symetric) {
    end.pos = Vec2d(-end.pos.x(), end.pos.y());
    end.theta = -end.theta;
    end.tangent = Vec2d(end.tangent.x(), -end.tangent.y());
  }

  // Check if out of required region.
  if (end.theta < 0.0 || end.theta > M_PI) {
    return false;
  }

  const double r_min = 1.0 / max_kappa;
  // Connect by a single line.
  constexpr double kEpsilon = 1e-6;
  if (end.theta < kEpsilon) {
    if (end.pos.y() > kEpsilon) return false;
    result->start = start;
    result->types = {GeometryPathType::STRAIGHT};
    result->lengths = {y_symetric ? -end.pos.x() : end.pos.x()};
    result->kappas = {0.0};
    result->ends = {goal};
    return true;
  }
  // Connect by a half_circle and a line, but we disable this.
  if (M_PI - end.theta < kEpsilon) {
    return false;
  }

  // Connect start and end.
  const double l2 =
      (end.pos.y() - r_min * (1.0 - end.tangent.x())) / end.tangent.y();
  const double l1 =
      end.pos.x() - r_min * end.tangent.y() - l2 * end.tangent.x();

  // Fill results.
  result->start = start;
  result->types = {
      GeometryPathType::STRAIGHT,
      x_symetric ? GeometryPathType::RIGHT : GeometryPathType::LEFT,
      GeometryPathType::STRAIGHT};
  result->kappas = {0.0, max_kappa, 0.0};
  if (y_symetric) {
    result->lengths = {-l1, -end.theta * r_min, -l2};
  } else {
    result->lengths = {l1, end.theta * r_min, l2};
  }
  const auto end1 = ExtendPathByConstantKappa(
      start, result->kappas[0], result->lengths[0], result->types[0]);
  const auto end2 = ExtendPathByConstantKappa(
      end1, result->kappas[1], result->lengths[1], result->types[1]);
  const auto end3 = ExtendPathByConstantKappa(
      end2, result->kappas[2], result->lengths[2], result->types[2]);
  result->ends = {end1, end2, end3};
  return true;
}

}  // namespace planner
}  // namespace qcraft
