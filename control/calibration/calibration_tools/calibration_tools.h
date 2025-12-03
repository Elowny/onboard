#ifndef ONBOARD_CONTROL_CALIBRATION_TOOLS_CALIBRATION_TOOLS_H_
#define ONBOARD_CONTROL_CALIBRATION_TOOLS_CALIBRATION_TOOLS_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "onboard/control/calibration/calibration_tools/calibration_interface.h"
#include "onboard/control/calibration/calibration_tools/calibration_utils.h"
#include "onboard/control/calibration/calibration_tools/proto/control_calibration.pb.h"
#include "onboard/math/filters/mean_filter.h"
#include "onboard/params/utils/control_calibration_check.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/chassis.pb.h"

namespace qcraft {
namespace control {
namespace calibration {

bool CheckCounterSuccess(double low, double up, const std::vector<double>& list,
                         int check_num);

class CalibrationTools {
 public:
  CalibrationTools(std::optional<std::string> dirname, const std::string& carid,
                   const std::string& method, double throttle_deadzone,
                   double brake_deadzone, bool is_save_sampledata);
  bool Process(double speed, double acceleration, double pitch_angle,
               double throttle, double brake, Chassis::GearPosition gear,
               vis::vantage::ChartDataProto* chart);
  void Reset();
  void SaveData();

 private:
  void PlotIdleProto(vis::vantage::ChartDataProto* chart);
  void PlotSlideProto(vis::vantage::ChartDataProto* chart);
  void PlotThrottleProto(vis::vantage::ChartDataProto* chart);
  void PlotBrakeProto(vis::vantage::ChartDataProto* chart);
  void PlotCheckTips(vis::vantage::ChartDataProto* chart);

  void UpdateIdleByFilter(const CalibrationProto& in_sample_proto,
                          const CalibrationProto& in_output_proto);
  void UpdateThrottleByFilter(const CalibrationProto& in_sample_proto,
                              const CalibrationProto& in_output_proto);
  void UpdateBrakeByFilter(const CalibrationProto& in_sample_proto,
                           const CalibrationProto& in_output_proto);

  void CheckThrottleSample();
  void CheckBrakeSample();

  CalibrationProto AddReverseIdleProto(const CalibrationProto& proto);
  CalibrationProto AddReverseThrottleProto(const CalibrationProto& proto);
  CalibrationProto AddReverseBrakeProto(const CalibrationProto& proto);

  bool IsMonotonicity();

  std::unique_ptr<MeanFilter> pitch_angle_filter_;
  std::unique_ptr<CalibrationInterface> calibration_method_;
  CalibrationCheck calibraion_checker_;

  Method method_;
  double throttle_deadzone_ = 0.0;
  double brake_deadzone_ = 0.0;

  CalibrationProto sample_proto_;
  CalibrationProto output_proto_;

  std::optional<std::string> filename_sample_ = std::nullopt;
  std::optional<std::string> filename_output_ = std::nullopt;

  bool is_save_sampledata_ = true;

  std::vector<std::vector<double>> check_throttle_x_, check_brake_x_;
  std::vector<std::vector<double>> check_throttle_y_, check_brake_y_;

  PolyFit idle_polyfit_;
  PolyFit throttle_polyfit_;
  PolyFit brake_polyfit_;
};

}  // namespace calibration
}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CALIBRATION_TOOLS_CALIBRATION_TOOLS_H_
