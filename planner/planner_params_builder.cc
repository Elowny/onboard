#include "onboard/planner/planner_params_builder.h"

#include <cmath>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/proto/aabox3d.pb.h"
#include "onboard/math/util.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/params/v2/proto/vehicle/installation.pb.h"
#include "onboard/perception/perception_flags.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/util/vehicle_util.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/proto_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {

std::string GetDefaultParamsFile(
    VehicleModel vehicle_model,
    VehicleInstallationProto::VehiclePlan vehicle_plan) {
  const std::string vehicle_model_str = [](VehicleModel vehicle_model) {
    // See go/vehicles
    switch (vehicle_model) {
      case VEHICLE_ZHONGXING:
      case VEHICLE_JINLV_MINIBUS:
      case VEHICLE_HIGER:
      case VEHICLE_ZHONGTONG55:
      case VEHICLE_DONGFENG:
      case VEHICLE_HIGER65_LONGZHOU_ONE:
      case VEHICLE_ZEV_LONGZHOU_ONE:
        return "robobus_6m_";
      case VEHICLE_ZHONGTONG:
      case VEHICLE_ZHONGTONG6:
      case VEHICLE_BYD:
      case VEHICLE_POLERSTAR:
      case VEHICLE_POLERSTAR_2:
      case VEHICLE_BYD_LONGZHOU_LONG:
      case VEHICLE_FOTON_LONGZHOU_LONG:
      case VEHICLE_HIGER85_LONGZHOU_LONG:
      case VEHICLE_JINLV85_LONGZHOU_LONG:
        return "robobus_10m_";
      case VEHICLE_UNKNOWN:
      case VEHICLE_PIXLOOP:
      case VEHICLE_SKYWELL:
      case VEHICLE_TEST_BENCH:
      case VEHICLE_LINCOLN_MKZ:
      case VEHICLE_LINCOLN_MKZ_AS_PACMOD:
      case VEHICLE_AION_LX:
      case VEHICLE_MARVELR:
      case VEHICLE_MARVELX:
      case VEHICLE_AION_LX_PLUS_SUV:
      case VEHICLE_MARVELR_NEW:
      case VEHICLE_QCRAFTVEHICLE_SUV:
      case VEHICLE_HAVAL_SUV:
      case VEHICLE_HYUNDAI_CAR:
      case VEHICLE_SF5_SUV:
      case VEHICLE_HYPER_GT_CAR:
      case VEHICLE_FUKANG_CAR:
      case VEHICLE_JETOUR_SUV:
      case VEHICLE_SERES_SF5_SUV:
      case VEHICLE_HYPER_AH8_SUV:
      // TODO(lidong): We should create a separate config file for
      // VEHICLE_SHUNFENG.
      case VEHICLE_SHUNFENG:
        return "planner_default_";
    }
  }(vehicle_model);

  const std::string vehicle_plan_str =
      [](VehicleInstallationProto::VehiclePlan vehicle_plan) {
        switch (vehicle_plan) {
          case VehicleInstallationProto::VP_PBQ_V2:
            return "vision_only_";
          case VehicleInstallationProto::VP_DBQ:
          case VehicleInstallationProto::VP_DBQ_V2:
          case VehicleInstallationProto::VP_DBQ_V3:
          case VehicleInstallationProto::VP_DBQ_V4:
          case VehicleInstallationProto::VP_PBQ:
          case VehicleInstallationProto::VP_PBQ_V1:
          case VehicleInstallationProto::VP_DPC:
          case VehicleInstallationProto::VP_CBQ:
          case VehicleInstallationProto::VP_CBQ_V1:
          case VehicleInstallationProto::VP_CBQ_V2:
            // For non-vision-only vehicle plans and X9/S32G platforms, we use
            // perception configuration to check whether to load vision-only
            // params.
            return FLAGS_use_pure_vision_approach &&
                           (FLAGS_planner_running_platform == 2 ||
                            FLAGS_planner_running_platform == 3)
                       ? "vision_only_"
                       : "";
        }
      }(vehicle_plan);

  return absl::StrCat(vehicle_model_str, vehicle_plan_str, "params.pb.txt");
}

absl::StatusOr<std::string> GetParamFilePathPrefix(int platform_id) {
  switch (platform_id) {
    case 0:  // IPC
    case 1:  // Orin
      return "onboard/planner/params/";

    case 2:  // X9
      return "onboard/planner/params/X9/";

    case 3:  // S32G
      return "onboard/planner/params/S32G/";

    default:
      return absl::NotFoundError("Unsupported platform id");
  }
}

// Protests if any field is missing.
absl::Status ValidateParams(const google::protobuf::Message& params) {
  const google::protobuf::Descriptor* descriptor = params.GetDescriptor();
  const google::protobuf::Reflection* reflection = params.GetReflection();
  for (int i = 0; i < descriptor->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = descriptor->field(i);
    if (!field->is_optional()) continue;
    if (!reflection->HasField(params, field)) {
      return absl::NotFoundError(
          absl::StrCat("Missing field: ", field->full_name()));
    }
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      RETURN_IF_ERROR(ValidateParams(reflection->GetMessage(params, field)));
    }
  }
  return absl::OkStatus();
}

void ComputeVehicleModelParamsOfPlanner(
    const VehicleGeometryParamsProto& vehicle_geo_params,
    VehicleModel vehicle_model,
    PlannerVehicleModelParamsProto* vehicle_models_params) {
  const double half_width = 0.5 * vehicle_geo_params.width();
  /*----------------------trajectory optimizer model----------------------*/
  vehicle_models_params->mutable_trajectory_optimizer_vehicle_model_params()
      ->clear_circles();
  vehicle_models_params->mutable_trajectory_optimizer_vehicle_model_params()
      ->clear_mirror_circles();
  {
    constexpr double kFrontCircleDefualtRadius = 0.3;  // m.
    // Rear axis center.
    const auto rac = vehicle_models_params
                         ->mutable_trajectory_optimizer_vehicle_model_params()
                         ->add_circles();
    rac->set_dist_to_rac(half_width - vehicle_geo_params.back_edge_to_center());
    rac->set_angle_to_axis(0.0);
    rac->set_radius(half_width);
    rac->set_type(VehicleCircleModelParamsProto::REAR_AXIS_CENTER);
    // Front axis center.
    const auto fac = vehicle_models_params
                         ->mutable_trajectory_optimizer_vehicle_model_params()
                         ->add_circles();
    fac->set_dist_to_rac(vehicle_geo_params.front_edge_to_center() -
                         half_width);
    fac->set_angle_to_axis(0.0);
    fac->set_radius(half_width);
    fac->set_type(VehicleCircleModelParamsProto::FRONT_AXIS_CENTER);
    // Middle axis center.
    constexpr double kLengthWidthRateThreshold = 3.0;
    // Add one more circle if vehicle is long.
    if (vehicle_geo_params.length() / vehicle_geo_params.width() >
        kLengthWidthRateThreshold) {
      const auto mfac =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_circles();
      const double dist_to_rac1 =
          2.0 / 3.0 * (vehicle_geo_params.front_edge_to_center() - half_width) +
          1.0 / 3.0 * (half_width - vehicle_geo_params.back_edge_to_center());
      mfac->set_dist_to_rac(dist_to_rac1);
      mfac->set_angle_to_axis(0.0);
      mfac->set_radius(half_width);
      mfac->set_type(VehicleCircleModelParamsProto::MID_AXIS_CENTER);
      const auto mrac =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_circles();
      const double dist_to_rac2 =
          1.0 / 3.0 * (vehicle_geo_params.front_edge_to_center() - half_width) +
          2.0 / 3.0 * (half_width - vehicle_geo_params.back_edge_to_center());
      mrac->set_dist_to_rac(dist_to_rac2);
      mrac->set_angle_to_axis(0.0);
      mrac->set_radius(half_width);
      mrac->set_type(VehicleCircleModelParamsProto::MID_AXIS_CENTER);
    } else {
      if (!vehicle_models_params->trajectory_optimizer_vehicle_model_params()
               .use_less_circles()) {
        const auto mac =
            vehicle_models_params
                ->mutable_trajectory_optimizer_vehicle_model_params()
                ->add_circles();
        mac->set_dist_to_rac(0.5 * (vehicle_geo_params.front_edge_to_center() -
                                    vehicle_geo_params.back_edge_to_center()));
        mac->set_angle_to_axis(0.0);
        mac->set_radius(half_width);
        mac->set_type(VehicleCircleModelParamsProto::MID_AXIS_CENTER);
      }
    }
    // Corners.
    if (!vehicle_models_params->trajectory_optimizer_vehicle_model_params()
             .use_less_circles()) {
      const double dist_to_front_corner =
          Hypot(half_width - kFrontCircleDefualtRadius,
                vehicle_geo_params.front_edge_to_center() -
                    kFrontCircleDefualtRadius);
      const double front_angle =
          std::atan2(half_width - kFrontCircleDefualtRadius,
                     vehicle_geo_params.front_edge_to_center() -
                         kFrontCircleDefualtRadius);
      // Front left corner.
      const auto flc = vehicle_models_params
                           ->mutable_trajectory_optimizer_vehicle_model_params()
                           ->add_circles();
      flc->set_dist_to_rac(dist_to_front_corner);
      flc->set_angle_to_axis(front_angle);
      flc->set_radius(kFrontCircleDefualtRadius);
      flc->set_type(VehicleCircleModelParamsProto::FRONT_LEFT_CORNER);
      // Front right corner.
      const auto frc = vehicle_models_params
                           ->mutable_trajectory_optimizer_vehicle_model_params()
                           ->add_circles();
      frc->set_dist_to_rac(dist_to_front_corner);
      frc->set_angle_to_axis(-front_angle);
      frc->set_radius(kFrontCircleDefualtRadius);
      frc->set_type(VehicleCircleModelParamsProto::FRONT_RIGHT_CORNER);
    }
    // Add circles for mirror.
    if (vehicle_geo_params.has_left_mirror() &&
        vehicle_geo_params.has_right_mirror()) {
      // Left mirror.
      const auto left_mirror_circle =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_mirror_circles();
      left_mirror_circle->set_dist_to_rac(
          Hypot(vehicle_geo_params.left_mirror().x(),
                vehicle_geo_params.left_mirror().y()));
      left_mirror_circle->set_angle_to_axis(
          std::atan2(vehicle_geo_params.left_mirror().y(),
                     vehicle_geo_params.left_mirror().x()));
      left_mirror_circle->set_radius(vehicle_geo_params.left_mirror().length() *
                                     0.5);
      left_mirror_circle->set_type(VehicleCircleModelParamsProto::LEFT_MIRROR);
      // Right mirror.
      const auto right_mirror_circle =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_mirror_circles();
      right_mirror_circle->set_dist_to_rac(
          Hypot(vehicle_geo_params.right_mirror().x(),
                vehicle_geo_params.right_mirror().y()));
      right_mirror_circle->set_angle_to_axis(
          std::atan2(vehicle_geo_params.right_mirror().y(),
                     vehicle_geo_params.right_mirror().x()));
      right_mirror_circle->set_radius(
          vehicle_geo_params.right_mirror().length() * 0.5);
      right_mirror_circle->set_type(
          VehicleCircleModelParamsProto::RIGHT_MIRROR);
    }
  }

  /*--------------------freespace vehicle octagon model--------------------*/
  const auto vehicle_octagon_params =
      vehicle_models_params->mutable_freespace_vehicle_octagon_model_params();
  if (vehicle_geo_params.has_left_mirror() &&
      vehicle_geo_params.has_right_mirror()) {
    vehicle_octagon_params->set_mirror_offset_x(
        vehicle_geo_params.left_mirror().x());
    vehicle_octagon_params->set_mirror_offset_y(
        vehicle_geo_params.left_mirror().y());
    vehicle_octagon_params->set_mirror_radius(
        vehicle_geo_params.left_mirror().length() * 0.5);
    vehicle_octagon_params->set_mirror_height(
        vehicle_geo_params.left_mirror().z() -
        vehicle_geo_params.left_mirror().height() * 0.5);
  } else {
    vehicle_octagon_params->set_consider_mirror(false);
  }
  // Specify vehicle octagon model params from CAD model.
  switch (vehicle_model) {
    case VEHICLE_LINCOLN_MKZ:
    case VEHICLE_LINCOLN_MKZ_AS_PACMOD: {
      vehicle_octagon_params->set_front_corner_side_length(0.41);
      vehicle_octagon_params->set_rear_corner_side_length(0.35);
    } break;
    case VEHICLE_MARVELR:
    case VEHICLE_MARVELX:
    case VEHICLE_MARVELR_NEW: {
      vehicle_octagon_params->set_front_corner_side_length(0.44);
      vehicle_octagon_params->set_rear_corner_side_length(0.32);
    } break;
    case VEHICLE_QCRAFTVEHICLE_SUV: {
      vehicle_octagon_params->set_front_corner_side_length(0.41);
      vehicle_octagon_params->set_rear_corner_side_length(0.37);
    } break;
    default:
      break;
  }

  /*----------------------freespace local smoother model----------------------*/
  vehicle_models_params->mutable_freespace_local_smoother_vehicle_model_params()
      ->clear_circles();
  vehicle_models_params->mutable_freespace_local_smoother_vehicle_model_params()
      ->clear_mirror_circles();
  // Copy mirror params from trajectory optimizer.
  vehicle_models_params->mutable_freespace_local_smoother_vehicle_model_params()
      ->mutable_mirror_circles()
      ->CopyFrom(
          vehicle_models_params->trajectory_optimizer_vehicle_model_params()
              .mirror_circles());
  {
    // Corners.
    constexpr double kFrontCircleDefualtRadius = 0.6;  // m.
    constexpr double kRearCircleDefualtRadius = 0.5;   // m.
    double front_circle_radius = kFrontCircleDefualtRadius;
    double rear_circle_radius = kRearCircleDefualtRadius;
    // Specify vehicle circle model params from CAD model.
    switch (vehicle_model) {
      case VEHICLE_LINCOLN_MKZ:
      case VEHICLE_LINCOLN_MKZ_AS_PACMOD: {
        front_circle_radius = 0.68;
        rear_circle_radius = 0.59;
      } break;
      case VEHICLE_MARVELR:
      case VEHICLE_MARVELX:
      case VEHICLE_MARVELR_NEW: {
        front_circle_radius = 0.73;
        rear_circle_radius = 0.54;
      } break;
      case VEHICLE_QCRAFTVEHICLE_SUV: {
        front_circle_radius = 0.68;
        rear_circle_radius = 0.63;
      } break;
      default:
        break;
    }
    // Front corners.
    const double dist_to_front_corner =
        Hypot(half_width - front_circle_radius,
              vehicle_geo_params.front_edge_to_center() - front_circle_radius);
    const double front_angle = std::atan2(
        half_width - front_circle_radius,
        vehicle_geo_params.front_edge_to_center() - front_circle_radius);
    // Front left corner.
    const auto flc =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    flc->set_dist_to_rac(dist_to_front_corner);
    flc->set_angle_to_axis(front_angle);
    flc->set_radius(front_circle_radius);
    flc->set_type(VehicleCircleModelParamsProto::FRONT_LEFT_CORNER);
    // Front right corner.
    const auto frc =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    frc->set_dist_to_rac(dist_to_front_corner);
    frc->set_angle_to_axis(-front_angle);
    frc->set_radius(front_circle_radius);
    frc->set_type(VehicleCircleModelParamsProto::FRONT_RIGHT_CORNER);
    // Rear corners.
    const double dist_to_rear_corner =
        Hypot(half_width - rear_circle_radius,
              vehicle_geo_params.back_edge_to_center() - rear_circle_radius);
    const double rear_angle = std::atan2(
        half_width - rear_circle_radius,
        vehicle_geo_params.back_edge_to_center() - rear_circle_radius);
    // Rear left corner.
    const auto rlc =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    rlc->set_dist_to_rac(dist_to_rear_corner);
    rlc->set_angle_to_axis(M_PI - rear_angle);
    rlc->set_radius(rear_circle_radius);
    rlc->set_type(VehicleCircleModelParamsProto::REAR_LEFT_CORNER);
    // Rear right corner.
    const auto rrc =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    rrc->set_dist_to_rac(dist_to_rear_corner);
    rrc->set_angle_to_axis(rear_angle - M_PI);
    rrc->set_radius(rear_circle_radius);
    rrc->set_type(VehicleCircleModelParamsProto::REAR_RIGHT_CORNER);
    // Rear axis center.
    const auto rac =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    rac->set_dist_to_rac(half_width + rear_circle_radius -
                         vehicle_geo_params.back_edge_to_center());
    rac->set_angle_to_axis(0.0);
    rac->set_radius(half_width);
    rac->set_type(VehicleCircleModelParamsProto::REAR_AXIS_CENTER);
    // Front axis center.
    const auto fac =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    fac->set_dist_to_rac(vehicle_geo_params.front_edge_to_center() -
                         half_width - front_circle_radius);
    fac->set_angle_to_axis(0.0);
    fac->set_radius(half_width);
    fac->set_type(VehicleCircleModelParamsProto::FRONT_AXIS_CENTER);
    // Add one more circle if vehicle is long.
    if (vehicle_geo_params.length() > 2.0 * vehicle_geo_params.width() +
                                          rear_circle_radius +
                                          front_circle_radius) {
      const auto mac =
          vehicle_models_params
              ->mutable_freespace_local_smoother_vehicle_model_params()
              ->add_circles();
      const double dist_to_mac =
          0.5 * (vehicle_geo_params.front_edge_to_center() -
                 vehicle_geo_params.back_edge_to_center() + rear_circle_radius -
                 front_circle_radius);
      mac->set_dist_to_rac(dist_to_mac);
      mac->set_angle_to_axis(0.0);
      mac->set_radius(half_width);
      mac->set_type(VehicleCircleModelParamsProto::MID_AXIS_CENTER);
    }
  }
  vehicle_models_params->set_is_vehicle_bus_model(IsBus(vehicle_model));
}

void FillPlannerFunctionsParams(
    VehicleModel vehicle_model,
    PlannerFunctionsParamsProto* planner_functions_params) {
  switch (vehicle_model) {
    case VEHICLE_LINCOLN_MKZ:
    case VEHICLE_LINCOLN_MKZ_AS_PACMOD:
    case VEHICLE_MARVELR:
    case VEHICLE_MARVELR_NEW:
    case VEHICLE_JINLV_MINIBUS:
      planner_functions_params->set_enable_three_point_turn(true);
      break;
      // TODO(Zhuang): Enable for WENJIE_M5 and GUANGQI.
    default:
      planner_functions_params->set_enable_three_point_turn(false);
      break;
  }
}

void FillAlccParamsMissingFieldsWithDefault(
    const PlannerParamsProto& default_planner_params,
    AlccTaskParamsProto* alcc_params) {
  // Fill est planner params.
  FillInMissingFieldsWithDefault(default_planner_params.speed_finder_params(),
                                 alcc_params->mutable_speed_finder_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_params(),
      alcc_params->mutable_trajectory_optimizer_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.decision_constraint_config(),
      alcc_params->mutable_decision_constraint_config());
  FillInMissingFieldsWithDefault(default_planner_params.initializer_params(),
                                 alcc_params->mutable_initializer_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      alcc_params->mutable_motion_constraint_params());
  FillInMissingFieldsWithDefault(default_planner_params.vehicle_models_params(),
                                 alcc_params->mutable_vehicle_models_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.planner_functions_params(),
      alcc_params->mutable_planner_functions_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.spacetime_planner_object_trajectories_params(),
      alcc_params->mutable_spacetime_planner_object_trajectories_params());

  // Fill lane change style params.
  FillInMissingFieldsWithDefault(
      default_planner_params.speed_finder_lc_radical_params(),
      alcc_params->mutable_speed_finder_lc_radical_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.speed_finder_lc_conservative_params(),
      alcc_params->mutable_speed_finder_lc_conservative_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_radical_params(),
      alcc_params->mutable_trajectory_optimizer_lc_radical_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_normal_params(),
      alcc_params->mutable_trajectory_optimizer_lc_normal_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_conservative_params(),
      alcc_params->mutable_trajectory_optimizer_lc_conservative_params());
}

// The document about params loading priority is
// https://qcraft.feishu.cn/docx/MY7idcAUro5ksixkEcKcJ3O6n1U.
void CreateDefaultParam(const VehicleGeometryParamsProto& vehicle_geo_params,
                        VehicleModel vehicle_model,
                        PlannerParamsProto* planner_params) {
  // Fill est planner with default speed finder and trajectory optimizer params.
  SpeedFinderParamsProto default_speed_finder_params;
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/speed_finder_default_params.pb.txt",
      &default_speed_finder_params));
  TrajectoryOptimizerParamsProto default_trajectory_optimizer_params;
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/trajectory_optimizer_default_params.pb.txt",
      &default_trajectory_optimizer_params));
  FillInMissingFieldsWithDefault(default_speed_finder_params,
                                 planner_params->mutable_speed_finder_params());
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      planner_params->mutable_trajectory_optimizer_params());

  // Fill est planner with default planner param.
  PlannerParamsProto default_planner_params;
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/planner_default_params.pb.txt",
      &default_planner_params));

  // Override default_planner_param with params which defined by platform and
  // default algorithm params.
  FillInMissingFieldsWithDefault(default_planner_params, planner_params);

  // Fill fallback planner.
  FillInMissingFieldsWithDefault(
      planner_params->speed_finder_params(),
      planner_params->mutable_fallback_planner_params()
          ->mutable_speed_finder_params());
  // Fill freespace planner.
  FillInMissingFieldsWithDefault(
      planner_params->speed_finder_params(),
      planner_params->mutable_freespace_params_for_parking()
          ->mutable_speed_finder_params());
  FillInMissingFieldsWithDefault(
      planner_params->speed_finder_params(),
      planner_params->mutable_freespace_params_for_driving()
          ->mutable_speed_finder_params());
  // Fill acc params.
  FillInMissingFieldsWithDefault(
      planner_params->speed_finder_params(),
      planner_params->mutable_acc_params()->mutable_speed_finder_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      planner_params->mutable_acc_params()->mutable_motion_constraint_params());
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/acc_req_params.pb.txt",
      planner_params->mutable_acc_params()->mutable_acc_req_params()));
  // Fill noa params.
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/noa_req_params.pb.txt",
      planner_params->mutable_noa_params()->mutable_noa_req_params()));
  // Fill default path_finder params into default planner params.
  FreespacePathFinderParamsProto default_path_finder_params;
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/path_finder_default_params.pb.txt",
      &default_path_finder_params));
  FillInMissingFieldsWithDefault(
      default_path_finder_params,
      planner_params->mutable_freespace_params_for_parking()
          ->mutable_path_finder_params());
  FillInMissingFieldsWithDefault(
      default_path_finder_params,
      planner_params->mutable_freespace_params_for_driving()
          ->mutable_path_finder_params());

  // Fill default local_smoother params into default planner params.
  FreespaceLocalSmootherParamsProto default_local_smoother_params;
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/local_smoother_default_params.pb.txt",
      &default_local_smoother_params));

  FillInMissingFieldsWithDefault(
      default_local_smoother_params,
      planner_params->mutable_freespace_params_for_parking()
          ->mutable_local_smoother_params());
  FillInMissingFieldsWithDefault(
      default_local_smoother_params,
      planner_params->mutable_freespace_params_for_driving()
          ->mutable_local_smoother_params());

  // Copy
  FillInMissingFieldsWithDefault(
      planner_params->motion_constraint_params(),
      planner_params->mutable_freespace_params_for_parking()
          ->mutable_motion_constraint_params());

  FillInMissingFieldsWithDefault(
      planner_params->motion_constraint_params(),
      planner_params->mutable_freespace_params_for_driving()
          ->mutable_motion_constraint_params());

  // Fill style params.
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/speed_finder_lc_radical_params.pb.txt",
      planner_params->mutable_speed_finder_lc_radical_params()));
  FillInMissingFieldsWithDefault(
      planner_params->speed_finder_params(),
      planner_params->mutable_speed_finder_lc_radical_params());

  QCHECK(file_util::FileToProto(
      "onboard/planner/params/speed_finder_lc_conservative_params.pb.txt",
      planner_params->mutable_speed_finder_lc_conservative_params()));
  FillInMissingFieldsWithDefault(
      planner_params->speed_finder_params(),
      planner_params->mutable_speed_finder_lc_conservative_params());

  QCHECK(file_util::FileToProto(
      "onboard/planner/params/trajectory_optimizer_lc_radical_params.pb.txt",
      planner_params->mutable_trajectory_optimizer_lc_radical_params()));
  FillInMissingFieldsWithDefault(
      planner_params->trajectory_optimizer_params(),
      planner_params->mutable_trajectory_optimizer_lc_radical_params());

  QCHECK(file_util::FileToProto(
      "onboard/planner/params/trajectory_optimizer_lc_normal_params.pb.txt",
      planner_params->mutable_trajectory_optimizer_lc_normal_params()));
  FillInMissingFieldsWithDefault(
      planner_params->trajectory_optimizer_params(),
      planner_params->mutable_trajectory_optimizer_lc_normal_params());

  QCHECK(file_util::FileToProto(
      "onboard/planner/params/"
      "trajectory_optimizer_lc_conservative_params.pb.txt",
      planner_params->mutable_trajectory_optimizer_lc_conservative_params()));
  FillInMissingFieldsWithDefault(
      planner_params->trajectory_optimizer_params(),
      planner_params->mutable_trajectory_optimizer_lc_conservative_params());

  // Fill vehicle model params.
  ComputeVehicleModelParamsOfPlanner(
      vehicle_geo_params, vehicle_model,
      planner_params->mutable_vehicle_models_params());

  // Fill planner functions params.This should determined by params or HMI in
  // the future.
  FillPlannerFunctionsParams(
      vehicle_model, planner_params->mutable_planner_functions_params());

  // Fill alcc params.
  AlccTaskParamsProto default_alcc_params;
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/alcc_default_params.pb.txt",
      &default_alcc_params));
  FillAlccParamsMissingFieldsWithDefault(*planner_params, &default_alcc_params);
  FillInMissingFieldsWithDefault(default_alcc_params,
                                 planner_params->mutable_alcc_params());
}

}  // namespace

absl::StatusOr<PlannerParamsProto> BuildPlannerParams(
    const VehicleGeometryParamsProto& vehicle_geo_params,
    VehicleModel vehicle_model,
    VehicleInstallationProto::VehiclePlan vehicle_plan) {
  const int platform_id = FLAGS_planner_running_platform;
  ASSIGN_OR_RETURN(const auto path_prefix, GetParamFilePathPrefix(platform_id));

  PlannerParamsProto planner_params;
  const bool load_file_success = file_util::FileToProto(
      absl::StrCat(path_prefix,
                   GetDefaultParamsFile(vehicle_model, vehicle_plan)),
      &planner_params);

  if (!load_file_success) {
    return absl::NotFoundError(absl::StrCat("Cannot find param file for ",
                                            VehicleModel_Name(vehicle_model),
                                            " on platform ", platform_id));
  }

  // Fill with default planner params.
  CreateDefaultParam(vehicle_geo_params, vehicle_model, &planner_params);

  // Check that all fields are set.
  RETURN_IF_ERROR(ValidateParams(planner_params));
  return planner_params;
}

}  // namespace planner
}  // namespace qcraft
