#include "onboard/planner/util/vehicle_geometry_util.h"

#include <cmath>

namespace qcraft {
namespace planner {

double ComputeCenterMaxCurvature(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  return std::tan(vehicle_drive_params.max_steer_angle() /
                  vehicle_drive_params.steer_ratio()) /
         vehicle_geometry_params.wheel_base();
}

double ComputeRelaxedCenterMaxCurvature(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  constexpr double kMaxCurvatureRelaxFactor = 1.13;
  return ComputeCenterMaxCurvature(vehicle_geometry_params,
                                   vehicle_drive_params) *
         kMaxCurvatureRelaxFactor;
}

Vec2d ComputeAvFrontLeftCorner(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  const Vec2d normal = tangent.Perp();
  return av_pos + tangent * vehicle_geometry_params.front_edge_to_center() +
         normal * vehicle_geometry_params.width() * 0.5;
}

Vec2d ComputeAvFrontRightCorner(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  const Vec2d normal = tangent.Perp();
  return av_pos + tangent * vehicle_geometry_params.front_edge_to_center() -
         normal * vehicle_geometry_params.width() * 0.5;
}

Vec2d ComputeAvFrontCenter(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  return av_pos + tangent * vehicle_geometry_params.front_edge_to_center();
}

Vec2d ComputeAvGeometryCenter(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  return ComputeAvGeometryCenter(av_pos, tangent, vehicle_geometry_params);
}

Vec2d ComputeAvGeometryCenter(
    const Vec2d& av_pos, const Vec2d& av_dir,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  return av_pos + av_dir * 0.5 *
                      (vehicle_geometry_params.front_edge_to_center() -
                       vehicle_geometry_params.back_edge_to_center());
}

Box2d ComputeAvBox(const Vec2d& av_pos, double av_theta,
                   const VehicleGeometryParamsProto& vehicle_geometry_params) {
  return ComputeAvBoxWithBuffer(av_pos, av_theta, vehicle_geometry_params,
                                /*length_buffer=*/0.0, /*width_buffer=*/0.0);
}

Box2d ComputeAvBoxWithBuffer(
    const Vec2d& av_xy, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    double length_buffer, double width_buffer) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  const double half_length =
      vehicle_geometry_params.length() * 0.5 + length_buffer;
  const double half_width =
      vehicle_geometry_params.width() * 0.5 + width_buffer;
  const double rac_to_center =
      half_length -
      (length_buffer + vehicle_geometry_params.back_edge_to_center());
  const Vec2d center = av_xy + tangent * rac_to_center;
  return Box2d(half_length, half_width, center, av_theta, tangent);
}

Box2d ComputeAvWheelBox(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  return ComputeAvWheelBoxWithBuffer(av_pos, av_theta, vehicle_geometry_params,
                                     /*length_buffer=*/0.0,
                                     /*width_buffer=*/0.0);
}

Box2d ComputeAvWheelBoxWithBuffer(
    const Vec2d& av_pose, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    double length_buffer, double width_buffer) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  const double half_length =
      vehicle_geometry_params.wheel_base() * 0.5 + length_buffer;
  const double half_width =
      vehicle_geometry_params.width() * 0.5 + width_buffer;
  const double rac_to_center = half_length - length_buffer;
  const Vec2d center = av_pose + tangent * rac_to_center;
  return Box2d(half_length, half_width, center, av_theta, tangent);
}
}  // namespace planner
}  // namespace qcraft
