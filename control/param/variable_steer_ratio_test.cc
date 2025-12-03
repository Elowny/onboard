#include "onboard/control/param/variable_steer_ratio.h"

#include <stddef.h>

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/util.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::control {
namespace {

TEST(VariableSteerRatio, MarvelRTest) {
  std::vector<std::string> car_id_vec = {"Q2401", "Q2502", "Q8001"};
  for (const auto& car_id : car_id_vec) {
    auto param_manager = CreateParamManagerFromCarId(car_id);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);
    const auto& vehicle_drive_params =
        run_params.vehicle_params().vehicle_drive_params();
    const auto vehicle_type =
        run_params.vehicle_params().vehicle_info().vehicle_interface();
    const double max_steer_angle = vehicle_drive_params.max_steer_angle();
    const double steer_ratio = vehicle_drive_params.steer_ratio();

    // Set ground truth.
    VehicleInfoProto::VehicleInterface vehicle_type_gt;
    std::vector<double> steer_angle_vec;
    std::vector<double> steer_ratio_gt_vec;
    if (car_id == "Q2401") {
      vehicle_type_gt = VehicleInfoProto::MARVELR_NEW;
      steer_angle_vec = {-500.0, -400.0, -200.0, -100.0, 0.0,
                         100.0,  200.0,  400.0,  500.0};
      steer_ratio_gt_vec = {14.5, 14.5, 15.1, 15.4, 15.4,
                            15.4, 15.1, 14.5, 14.5};
    } else if (car_id == "Q2502") {
      vehicle_type_gt = VehicleInfoProto::QCRAFTVEHICLE_SUV;
      steer_angle_vec = {-500.0, -400.0, -300.0, -120.0, 0.0,
                         120.0,  300.0,  400.0,  500.0};
      steer_ratio_gt_vec = {15.2, 15.2, 15.8, 16.8, 16.8,
                            16.8, 15.8, 15.2, 15.2};
    } else if (car_id == "Q8001") {
      vehicle_type_gt = VehicleInfoProto::JINLV_MINIBUS;
      steer_angle_vec = {r2d(-max_steer_angle), 0.0, r2d(max_steer_angle)};
      steer_ratio_gt_vec = {steer_ratio, steer_ratio, steer_ratio};
    }

    // 1. Vehicle type validate.
    EXPECT_EQ(vehicle_type, vehicle_type_gt) << "car_id: " << car_id;
    VariableSteerRatio variable_steer_ratio(
        vehicle_drive_params,
        std::make_optional<VehicleInfoProto::VehicleInterface>(vehicle_type));
    // 2. Test data validate.
    EXPECT_EQ(steer_angle_vec.size(), steer_ratio_gt_vec.size())
        << "car_id: " << car_id;
    // 3. Steer ratio validate.
    for (size_t i = 0; i < steer_angle_vec.size(); ++i) {
      const double steer_ratio =
          variable_steer_ratio.steer_ratio(d2r(steer_angle_vec[i]));
      EXPECT_NEAR(steer_ratio, steer_ratio_gt_vec[i], 0.1)
          << "car_id: " << car_id << ", steer_angle: " << steer_angle_vec[i];
    }
    // 4. FrontWheelAngleToSteerAngle and SteerAngleToFrontWheelAngle validate.
    for (size_t i = 0; i < steer_angle_vec.size(); ++i) {
      const double front_wheel_angle =
          d2r(steer_angle_vec[i]) / steer_ratio_gt_vec[i];
      EXPECT_NEAR(
          variable_steer_ratio.FrontWheelAngleToSteerAngle(front_wheel_angle),
          d2r(steer_angle_vec[i]), 0.2)
          << "car_id: " << car_id << ", steer_angle: " << steer_angle_vec[i];
      EXPECT_NEAR(variable_steer_ratio.SteerAngleToFrontWheelAngle(
                      d2r(steer_angle_vec[i])),
                  front_wheel_angle, 0.01)
          << "car_id: " << car_id << ", steer_angle: " << steer_angle_vec[i];
    }
  }
}

}  // namespace
}  // namespace qcraft::control
