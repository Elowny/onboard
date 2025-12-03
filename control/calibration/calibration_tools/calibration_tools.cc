#include "onboard/control/calibration/calibration_tools/calibration_tools.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <utility>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/control/calibration/calibration_tools/calibration_force.h"
#include "onboard/control/calibration/calibration_tools/calibration_idle.h"
#include "onboard/control/calibration/calibration_tools/calibration_slide.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/proto/piecewise_linear_function.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace control {
namespace calibration {

void ExportProtoToSubchart(
    const std::vector<double>& x, const std::vector<std::vector<double>>& y,
    const std::string& x_name, const std::vector<std::string>& y_names,
    const vis::Color& color,
    const vis::vantage::ChartSeriesDataProto::PenStyle& pen_style,
    vis::vantage::ChartDataProto* chart) {
  QCHECK_NOTNULL(chart);
  CHECK_EQ(y.size(), y_names.size());

  auto* subchart = chart->add_subcharts();
  subchart->set_x_name(x_name);
  for (size_t i = 0; i < y.size(); ++i) {
    auto* y_elem = subchart->add_y();
    y_elem->set_name(y_names[i]);
    color.ToProto(y_elem->mutable_color());
    y_elem->set_pen_style(pen_style);
    for (size_t j = 0; j < y[i].size(); ++j) {
      subchart->add_x_values(x[j]);
      y_elem->add_values(y[i][j]);
    }
  }
}

bool CheckCounterSuccess(double low, double up, const std::vector<double>& list,
                         int check_num) {
  int seq = 0;
  for (const auto& elem : list) {
    if (elem >= low && elem <= up) seq++;
    if (seq > check_num) return true;
  }
  return false;
}

bool compare1d_max(std::pair<double, double> a, std::pair<double, double> b) {
  return a.first > b.first;
}
bool compare1d_min(std::pair<double, double> a, std::pair<double, double> b) {
  return a.first < b.first;
}

CalibrationTools::CalibrationTools(std::optional<std::string> dirname,
                                   const std::string& carid,
                                   const std::string& method,
                                   double throttle_deadzone,
                                   double brake_deadzone,
                                   bool is_save_sampledata)
    : idle_polyfit_(kIdlePolyfitOrder),
      throttle_polyfit_(kForcePolyfitOrder),
      brake_polyfit_(kForcePolyfitOrder) {
  QCHECK((method == "IDLE") || (method == "SLIDE") || (method == "FORCE"))
      << "method is not set !";

  throttle_deadzone_ = throttle_deadzone;
  brake_deadzone_ = brake_deadzone;

  constexpr int kWindowSize = 10;
  pitch_angle_filter_ = std::make_unique<MeanFilter>(kWindowSize);

  is_save_sampledata_ = is_save_sampledata && dirname;
  std::optional<std::string> file = std::nullopt;

  if (dirname) {
    std::filesystem::create_directories(dirname.value());
    QCHECK(std::filesystem::exists(dirname.value()))
        << "fail to build save dir as: " << dirname.value();
    file = absl::StrCat(dirname.value(), "/", carid);
  }

  if (method == "IDLE") {
    method_ = Method::IDLE;
    calibration_method_ = std::make_unique<CalibrationIdle>();
    QCHECK(calibration_method_->Init(file, throttle_deadzone, brake_deadzone));
    if (!dirname) return;
    filename_sample_ = absl::StrCat(file.value(), "_idle_sample.pb.txt");
    filename_output_ = absl::StrCat(file.value(), "_idle.pb.txt");
  } else if (method == "SLIDE") {
    method_ = Method::SLIDE;
    calibration_method_ = std::make_unique<CalibrationSlide>();
    QCHECK(calibration_method_->Init(file, throttle_deadzone, brake_deadzone));
    if (!dirname) return;
    filename_sample_ = absl::StrCat(file.value(), "_slide_sample.pb.txt");
    filename_output_ = absl::StrCat(file.value(), "_slide.pb.txt");
  } else if (method == "FORCE") {
    method_ = Method::FORCE;
    calibration_method_ = std::make_unique<CalibrationForce>();
    QCHECK(calibration_method_->Init(file, throttle_deadzone, brake_deadzone));
    if (!dirname) return;
    filename_sample_ = absl::StrCat(file.value(), "_force_sample.pb.txt");
    filename_output_ = absl::StrCat(file.value(), "_force.pb.txt");
  } else {
    LOG(FATAL) << "no method or wrong method set.";
  }
}

void CalibrationTools::Reset() {
  calibration_method_->Reset();
  sample_proto_.Clear();
  output_proto_.Clear();
  check_throttle_x_.clear();
  check_throttle_y_.clear();
  check_brake_x_.clear();
  check_brake_y_.clear();
}

bool CalibrationTools::Process(double speed, double acceleration,
                               double pitch_angle, double throttle,
                               double brake, Chassis::GearPosition gear,
                               vis::vantage::ChartDataProto* chart) {
  // Filter pitch angle.
  const double pitch_angle_smooth = pitch_angle_filter_->Update(pitch_angle);

  // Pitch angle < 0 is clamp slope.
  constexpr double kGravityAcceleration = 9.80665;
  const double acceleration_offset =
      -kGravityAcceleration * std::sin(pitch_angle_smooth);
  const double acceleration_pure = acceleration + acceleration_offset;

  // Speed truncate.
  constexpr double kEpsilonSpeed = 0.01;
  if (gear == Chassis::GEAR_REVERSE) {
    speed = std::min(speed, -kEpsilonSpeed);
  } else {
    speed = std::max(speed, kEpsilonSpeed);
  }

  // Calibration process.
  if (!calibration_method_->Process(speed, acceleration_pure, throttle,
                                    brake)) {
    return false;
  }

  // Get sample and out datas.
  sample_proto_ = calibration_method_->GetSampleProto();
  output_proto_ = calibration_method_->GetOutputProto();

  // Vantage plot datas.
  switch (method_) {
    case Method::IDLE:
      PlotIdleProto(chart);
      break;
    case Method::SLIDE:
      PlotSlideProto(chart);
      break;
    case Method::FORCE:
      PlotThrottleProto(chart);
      PlotBrakeProto(chart);
      CheckThrottleSample();
      CheckBrakeSample();
      PlotCheckTips(chart);
      break;
  }
  return true;
}

void CalibrationTools::SaveData() {
  if (sample_proto_.has_idle_v_a_plf() && output_proto_.has_idle_v_a_plf()) {
    UpdateIdleByFilter(sample_proto_, output_proto_);
  }
  if (sample_proto_.has_a_throttle_plf() &&
      output_proto_.has_a_throttle_plf()) {
    UpdateThrottleByFilter(sample_proto_, output_proto_);
  }
  if (sample_proto_.has_a_brake_plf() && output_proto_.has_a_brake_plf()) {
    UpdateBrakeByFilter(sample_proto_, output_proto_);
  }

  if (method_ == Method::FORCE) {
    if (output_proto_.has_idle_v_a_plf()) {
      const auto idle_proto = AddReverseIdleProto(output_proto_);
      output_proto_.mutable_idle_v_a_plf()->CopyFrom(idle_proto.idle_v_a_plf());
    }

    if (output_proto_.has_a_throttle_plf()) {
      const auto throttle_proto = AddReverseThrottleProto(output_proto_);
      output_proto_.mutable_a_throttle_plf()->CopyFrom(
          throttle_proto.a_throttle_plf());
    }

    if (output_proto_.has_a_brake_plf()) {
      const auto brake_proto = AddReverseBrakeProto(output_proto_);
      output_proto_.mutable_a_brake_plf()->CopyFrom(brake_proto.a_brake_plf());
    }
  }

  if (is_save_sampledata_ && filename_sample_) {
    if (!file_util::ProtoToTextFile(sample_proto_, filename_sample_.value())) {
      LOG(ERROR) << "Save sample data failed !";
    }
  } else {
    LOG(INFO) << "Print sample data result: " << sample_proto_.DebugString();
  }

  if (filename_output_ &&
      !file_util::ProtoToTextFile(output_proto_, filename_output_.value())) {
    LOG(ERROR) << "Save output data failed !";
  } else {
    LOG(INFO) << "Print output data result: " << output_proto_.DebugString();
  }
  VLOG(1) << "Save calibration data success !";

  // Check calibration is monotonicity
  if (!IsMonotonicity()) {
    LOG(ERROR) << "Check monotonicity failed, must manually modify result";
  }
}

CalibrationProto CalibrationTools::AddReverseIdleProto(
    const CalibrationProto& in_proto) {
  CalibrationProto out_proto;
  auto* idle_v_a_proto = out_proto.mutable_idle_v_a_plf();
  const auto idle_v_a_plf =
      PiecewiseLinearFunctionFromProto(in_proto.idle_v_a_plf());
  idle_v_a_proto->add_x(-3.0);
  idle_v_a_proto->add_y(-PrecisionTruncation(idle_v_a_plf.Evaluate(3.0), 3));
  idle_v_a_proto->add_x(-1.5);
  idle_v_a_proto->add_y(-PrecisionTruncation(idle_v_a_plf.Evaluate(1.5), 3));
  idle_v_a_proto->add_x(-0.01);
  idle_v_a_proto->add_y(-PrecisionTruncation(idle_v_a_plf.Evaluate(0.01), 3));
  for (int i = 0; i < in_proto.idle_v_a_plf().x_size(); i++) {
    idle_v_a_proto->add_x(in_proto.idle_v_a_plf().x().Get(i));
    idle_v_a_proto->add_y(in_proto.idle_v_a_plf().y().Get(i));
  }
  return out_proto;
}

CalibrationProto CalibrationTools::AddReverseThrottleProto(
    const CalibrationProto& in_proto) {
  CalibrationProto out_proto;
  auto* a_throttle_proto = out_proto.mutable_a_throttle_plf();
  const auto a_throttle_plf =
      PiecewiseLinearFunctionFromProto(in_proto.a_throttle_plf());
  for (size_t i = in_proto.a_throttle_plf().x_size() - 1; i > 0; i--) {
    const double x = in_proto.a_throttle_plf().x().Get(i);
    if (x != 0.0) {
      a_throttle_proto->add_x(-in_proto.a_throttle_plf().x().Get(i));
      a_throttle_proto->add_y(in_proto.a_throttle_plf().y().Get(i));
    }
  }
  a_throttle_proto->add_x(-0.001);
  a_throttle_proto->add_y(throttle_deadzone_);
  a_throttle_proto->add_x(0.001);
  a_throttle_proto->add_y(throttle_deadzone_);
  for (int i = 0; i < in_proto.a_throttle_plf().x_size(); i++) {
    const auto x = in_proto.a_throttle_plf().x().Get(i);
    if (x != 0.0) {
      a_throttle_proto->add_x(in_proto.a_throttle_plf().x().Get(i));
      a_throttle_proto->add_y(in_proto.a_throttle_plf().y().Get(i));
    }
  }
  return out_proto;
}

CalibrationProto CalibrationTools::AddReverseBrakeProto(
    const CalibrationProto& in_proto) {
  CalibrationProto out_proto;
  auto* a_brake_proto = out_proto.mutable_a_brake_plf();
  const auto a_brake_plf =
      PiecewiseLinearFunctionFromProto(in_proto.a_brake_plf());
  for (int i = 0; i < in_proto.a_brake_plf().x_size(); i++) {
    const double x = in_proto.a_brake_plf().x().Get(i);
    if (x != 0.0) {
      a_brake_proto->add_x(in_proto.a_brake_plf().x().Get(i));
      a_brake_proto->add_y(in_proto.a_brake_plf().y().Get(i));
    }
  }
  a_brake_proto->add_x(-0.001);
  a_brake_proto->add_y(brake_deadzone_);
  a_brake_proto->add_x(0.001);
  a_brake_proto->add_y(brake_deadzone_);
  for (size_t i = in_proto.a_brake_plf().x_size() - 1; i > 0; i--) {
    const double x = in_proto.a_brake_plf().x().Get(i);
    if (x != 0.0) {
      a_brake_proto->add_x(-in_proto.a_brake_plf().x().Get(i));
      a_brake_proto->add_y(in_proto.a_brake_plf().y().Get(i));
    }
  }
  return out_proto;
}

void CalibrationTools::CheckThrottleSample() {
  if (!sample_proto_.has_a_throttle_plf()) {
    return;
  }
  const auto plf_a_sample_x = sample_proto_.a_throttle_plf().x();
  const auto plf_a_sample_y = sample_proto_.a_throttle_plf().y();
  std::vector<double> a_sample_x(plf_a_sample_x.begin(), plf_a_sample_x.end());
  std::vector<double> a_sample_y(plf_a_sample_y.begin(), plf_a_sample_y.end());
  if (a_sample_x.empty()) {
    return;
  }
  std::sort(a_sample_x.begin(), a_sample_x.end());
  const auto min = std::min_element(a_sample_y.begin(), a_sample_y.end());
  const auto max = std::max_element(a_sample_y.begin(), a_sample_y.end());

  check_throttle_x_.clear();
  check_throttle_y_.clear();

  const std::vector<double> list = {0.0, 1.0, 2.0, 3.0, 4.0};
  constexpr int kCheckNum = 25;

  for (size_t i = 0; i < list.size() - 1; i++) {
    const bool success =
        CheckCounterSuccess(list[i], list[i + 1], a_sample_x, kCheckNum);
    if (!success) {
      std::vector<double> limit = {list[i], list[i], list[i + 1], list[i + 1],
                                   list[i]};
      check_throttle_x_.push_back(std::move(limit));
    }
  }
  for (size_t i = 0; i < check_throttle_x_.size(); i++) {
    std::vector<double> value = {*min, *max, *max, *min, *min};
    check_throttle_y_.push_back(std::move(value));
  }
}

void CalibrationTools::CheckBrakeSample() {
  if (!sample_proto_.has_a_brake_plf()) {
    return;
  }
  const auto plf_a_sample_x = sample_proto_.a_brake_plf().x();
  const auto plf_a_sample_y = sample_proto_.a_brake_plf().y();
  std::vector<double> a_sample_x(plf_a_sample_x.begin(), plf_a_sample_x.end());
  std::vector<double> a_sample_y(plf_a_sample_y.begin(), plf_a_sample_y.end());
  if (a_sample_x.empty()) {
    return;
  }
  std::sort(a_sample_x.begin(), a_sample_x.end());
  const auto min = std::min_element(a_sample_y.begin(), a_sample_y.end());
  const auto max = std::max_element(a_sample_y.begin(), a_sample_y.end());

  check_brake_x_.clear();
  check_brake_y_.clear();

  const std::vector<double> list = {-5.0, -4.0, -3.0, -2.0, -1.0, 0.0};
  constexpr int kCheckNum = 25;

  for (size_t i = 0; i < list.size() - 1; i++) {
    const bool success =
        CheckCounterSuccess(list[i], list[i + 1], a_sample_x, kCheckNum);
    if (!success) {
      std::vector<double> limit = {list[i], list[i], list[i + 1], list[i + 1],
                                   list[i]};
      check_brake_x_.push_back(std::move(limit));
    }
  }
  for (size_t i = 0; i < check_brake_x_.size(); i++) {
    std::vector<double> value = {*min, *max, *max, *min, *min};
    check_brake_y_.push_back(std::move(value));
  }
}

void CalibrationTools::PlotIdleProto(vis::vantage::ChartDataProto* chart) {
  if (!sample_proto_.has_idle_v_a_plf()) return;
  chart->set_title("control_calibration");
  const auto plf_v_sample = sample_proto_.idle_v_a_plf().x();
  const auto plf_a_sample = sample_proto_.idle_v_a_plf().y();
  std::vector<std::pair<double, double>> vec_pair;
  vec_pair.reserve(plf_v_sample.size());
  for (int i = 0; i < plf_v_sample.size(); i++) {
    vec_pair.push_back({plf_v_sample.Get(i), plf_a_sample.Get(i)});
  }
  std::sort(vec_pair.begin(), vec_pair.end(), compare1d_min);
  std::vector<double> v_sample;
  std::vector<double> a_sample;
  v_sample.reserve(vec_pair.size());
  a_sample.reserve(a_sample.size());
  for (size_t i = 0; i < vec_pair.size(); i++) {
    v_sample.push_back(vec_pair[i].first);
    a_sample.push_back(vec_pair[i].second);
  }
  ExportProtoToSubchart(v_sample, {a_sample}, "speed(m/s)",
                        {"acc_sample(m/s2)"}, vis::Color::kOrange,
                        vis::vantage::ChartSeriesDataProto::DOTLINE, chart);

  if (!output_proto_.has_idle_v_a_plf()) return;
  const auto plf_v_out = output_proto_.idle_v_a_plf().x();
  const auto plf_a_out = output_proto_.idle_v_a_plf().y();
  std::vector<double> v_out(plf_v_out.begin(), plf_v_out.end());
  std::vector<double> a_out(plf_a_out.begin(), plf_a_out.end());
  ExportProtoToSubchart(v_out, {a_out}, "speed(m/s)", {"acc(m/s2)"},
                        vis::Color::kDarkGreen,
                        vis::vantage::ChartSeriesDataProto::SOLIDLINE, chart);
}

void CalibrationTools::PlotSlideProto(vis::vantage::ChartDataProto* chart) {
  if (!sample_proto_.has_idle_v_a_plf()) return;
  chart->set_title("control_calibration");
  const auto plf_v_sample = sample_proto_.idle_v_a_plf().x();
  const auto plf_a_sample = sample_proto_.idle_v_a_plf().y();
  std::vector<std::pair<double, double>> vec_pair;
  vec_pair.reserve(plf_v_sample.size());
  for (int i = 0; i < plf_v_sample.size(); i++) {
    vec_pair.push_back({plf_v_sample.Get(i), plf_a_sample.Get(i)});
  }
  std::sort(vec_pair.begin(), vec_pair.end(), compare1d_max);
  std::vector<double> v_sample;
  std::vector<double> a_sample;
  v_sample.reserve(vec_pair.size());
  a_sample.reserve(vec_pair.size());
  for (const auto& p : vec_pair) {
    v_sample.push_back(p.first);
    a_sample.push_back(p.second);
  }
  ExportProtoToSubchart(v_sample, {a_sample}, "speed(m/s)",
                        {"acc_sample(m/s2)"}, vis::Color::kOrange,
                        vis::vantage::ChartSeriesDataProto::DOTLINE, chart);

  if (!output_proto_.has_idle_v_a_plf()) return;
  const auto plf_v_out = output_proto_.idle_v_a_plf().x();
  const auto plf_a_out = output_proto_.idle_v_a_plf().y();
  std::vector<double> v_out(plf_v_out.begin(), plf_v_out.end());
  std::vector<double> a_out(plf_a_out.begin(), plf_a_out.end());
  ExportProtoToSubchart(v_out, {a_out}, "speed(m/s)", {"acc(m/s2)"},
                        vis::Color::kDarkGreen,
                        vis::vantage::ChartSeriesDataProto::SOLIDLINE, chart);
}

void CalibrationTools::PlotThrottleProto(vis::vantage::ChartDataProto* chart) {
  if (!sample_proto_.has_a_throttle_plf()) return;
  chart->set_title("control_calibration");
  const auto plf_a_sample = sample_proto_.a_throttle_plf().x();
  const auto plf_throttle_sample = sample_proto_.a_throttle_plf().y();

  std::vector<std::pair<double, double>> vec_pair;
  vec_pair.reserve(plf_a_sample.size());
  for (int i = 0; i < plf_a_sample.size(); i++) {
    vec_pair.push_back({plf_a_sample.Get(i), plf_throttle_sample.Get(i)});
  }
  std::sort(vec_pair.begin(), vec_pair.end(), compare1d_max);
  std::vector<double> a_sample;
  std::vector<double> throttle_sample;
  a_sample.reserve(vec_pair.size());
  throttle_sample.reserve(vec_pair.size());
  for (size_t i = 0; i < vec_pair.size(); i++) {
    a_sample.push_back(vec_pair[i].first);
    throttle_sample.push_back(vec_pair[i].second);
  }
  ExportProtoToSubchart(a_sample, {throttle_sample}, "acc(m/s2)",
                        {"throttle_sample(%)"}, vis::Color::kOrange,
                        vis::vantage::ChartSeriesDataProto::DOTLINE, chart);

  if (!sample_proto_.has_a_throttle_plf()) return;
  const auto plf_a_out = output_proto_.a_throttle_plf().x();
  const auto plf_throttle_out = output_proto_.a_throttle_plf().y();
  std::vector<double> a_out(plf_a_out.begin(), plf_a_out.end());
  std::vector<double> throttle_out(plf_throttle_out.begin(),
                                   plf_throttle_out.end());
  ExportProtoToSubchart(a_out, {throttle_out}, "acc(m/s2)", {"throttle(%)"},
                        vis::Color::kDarkGreen,
                        vis::vantage::ChartSeriesDataProto::SOLIDLINE, chart);
}

void CalibrationTools::PlotBrakeProto(vis::vantage::ChartDataProto* chart) {
  if (!sample_proto_.has_a_brake_plf()) return;
  chart->set_title("control_calibration");
  const auto plf_a_sample = sample_proto_.a_brake_plf().x();
  const auto plf_brake_sample = sample_proto_.a_brake_plf().y();

  std::vector<std::pair<double, double>> vec_pair;
  vec_pair.reserve(plf_a_sample.size());
  for (int i = 0; i < plf_a_sample.size(); i++) {
    vec_pair.push_back({plf_a_sample.Get(i), plf_brake_sample.Get(i)});
  }
  std::sort(vec_pair.begin(), vec_pair.end(), compare1d_max);
  std::vector<double> a_sample;
  std::vector<double> brake_sample;
  a_sample.reserve(vec_pair.size());
  brake_sample.reserve(vec_pair.size());
  for (size_t i = 0; i < vec_pair.size(); i++) {
    a_sample.push_back(vec_pair[i].first);
    brake_sample.push_back(vec_pair[i].second);
  }
  ExportProtoToSubchart(a_sample, {brake_sample}, "acc(m/s2)",
                        {"brake_sample(%)"}, vis::Color::kOrange,
                        vis::vantage::ChartSeriesDataProto::DOTLINE, chart);

  if (!output_proto_.has_a_brake_plf()) return;
  const auto plf_a_out = output_proto_.a_brake_plf().x();
  const auto plf_brake_out = output_proto_.a_brake_plf().y();
  std::vector<double> a_out(plf_a_out.begin(), plf_a_out.end());
  std::vector<double> brake_out(plf_brake_out.begin(), plf_brake_out.end());
  ExportProtoToSubchart(a_out, {brake_out}, "acc(m/s2)", {"brake(%)"},
                        vis::Color::kDarkGreen,
                        vis::vantage::ChartSeriesDataProto::SOLIDLINE, chart);
}

bool CalibrationTools::IsMonotonicity() {
  bool check_idle = true;
  if (output_proto_.has_idle_v_a_plf()) {
    const auto idle_v_a_plf =
        PiecewiseLinearFunctionFromProto(output_proto_.idle_v_a_plf());

    check_idle = calibraion_checker_.SetCheck(idle_v_a_plf) &&
                 calibraion_checker_.CheckIdlePLF();
  }

  bool check_slide = true;
  if (output_proto_.has_a_throttle_plf()) {
    const auto a_throttle_plf =
        PiecewiseLinearFunctionFromProto(output_proto_.a_throttle_plf());

    check_slide = calibraion_checker_.SetCheck(a_throttle_plf) &&
                  calibraion_checker_.CheckThrottlePLF();
  }

  bool check_force = true;
  if (output_proto_.has_a_brake_plf()) {
    const auto a_brake_plf =
        PiecewiseLinearFunctionFromProto(output_proto_.a_brake_plf());

    check_force = calibraion_checker_.SetCheck(a_brake_plf) &&
                  calibraion_checker_.CheckBrakePLF();
  }

  return check_idle && check_slide && check_force;
}

void CalibrationTools::PlotCheckTips(vis::vantage::ChartDataProto* chart) {
  if (!check_throttle_x_.empty() && !check_throttle_y_.empty() &&
      check_throttle_x_.size() == check_throttle_y_.size()) {
    for (size_t i = 0; i < check_throttle_x_.size(); ++i) {
      ExportProtoToSubchart(check_throttle_x_[i], {check_throttle_y_[i]},
                            "acc(m/s2)", {" "}, vis::Color::kRed,
                            vis::vantage::ChartSeriesDataProto::SOLIDLINE,
                            chart);
    }
  }
  if (!check_brake_x_.empty() && !check_brake_y_.empty() &&
      check_brake_x_.size() == check_brake_y_.size()) {
    for (size_t i = 0; i < check_brake_x_.size(); ++i) {
      ExportProtoToSubchart(check_brake_x_[i], {check_brake_y_[i]}, "acc(m/s2)",
                            {" "}, vis::Color::kRed,
                            vis::vantage::ChartSeriesDataProto::SOLIDLINE,
                            chart);
    }
  }
}

void CalibrationTools::UpdateIdleByFilter(
    const CalibrationProto& in_sample_proto,
    const CalibrationProto& in_output_proto) {
  const auto idle_v_a_plf =
      PiecewiseLinearFunctionFromProto(in_output_proto.idle_v_a_plf());

  CalibrationProto out_sample;
  auto* idle_v_a_sample = out_sample.mutable_idle_v_a_plf();
  std::vector<double> x, y;
  constexpr double kIdleAccLimit = 0.1;
  for (int i = 0; i < in_sample_proto.idle_v_a_plf().x_size(); ++i) {
    const double y_sample = in_sample_proto.idle_v_a_plf().y().Get(i);
    const double y_evaluate =
        idle_v_a_plf.Evaluate(in_sample_proto.idle_v_a_plf().x().Get(i));
    if (std::fabs(y_sample - y_evaluate) < kIdleAccLimit) {
      x.push_back(in_sample_proto.idle_v_a_plf().x().Get(i));
      y.push_back(in_sample_proto.idle_v_a_plf().y().Get(i));
      idle_v_a_sample->add_x(x.back());
      idle_v_a_sample->add_y(y.back());
    }
  }
  if (!idle_polyfit_.ComputerCoff(x, y)) return;

  CalibrationProto out_proto;
  auto* idle_v_a_proto = out_proto.mutable_idle_v_a_plf();
  for (int i = 0; i < in_output_proto.idle_v_a_plf().x_size(); i++) {
    const double x_out = in_output_proto.idle_v_a_plf().x().Get(i);
    idle_v_a_proto->add_x(x_out);
    idle_v_a_proto->add_y(idle_polyfit_.GetPolyVal(x_out));
  }

  sample_proto_.mutable_idle_v_a_plf()->CopyFrom(out_sample.idle_v_a_plf());
  output_proto_.mutable_idle_v_a_plf()->CopyFrom(out_proto.idle_v_a_plf());
}

void CalibrationTools::UpdateThrottleByFilter(
    const CalibrationProto& in_sample_proto,
    const CalibrationProto& in_output_proto) {
  const auto a_thtottle_plf =
      PiecewiseLinearFunctionFromProto(in_output_proto.a_throttle_plf());

  CalibrationProto out_sample;
  auto* a_throttle_sample = out_sample.mutable_a_throttle_plf();
  std::vector<double> x, y;
  constexpr double kThrottleLimit = 5.0;
  for (int i = 0; i < in_sample_proto.a_throttle_plf().x_size(); ++i) {
    const double y_sample = in_sample_proto.a_throttle_plf().y().Get(i);
    const double y_evaluate =
        a_thtottle_plf.Evaluate(in_sample_proto.a_throttle_plf().x().Get(i));
    if (std::fabs(y_sample - y_evaluate) < kThrottleLimit) {
      x.push_back(in_sample_proto.a_throttle_plf().x().Get(i));
      y.push_back(in_sample_proto.a_throttle_plf().y().Get(i));
      a_throttle_sample->add_x(x.back());
      a_throttle_sample->add_y(y.back());
    }
  }
  if (!throttle_polyfit_.ComputerCoff(x, y)) return;

  CalibrationProto out_proto;
  auto* a_throttle_proto = out_proto.mutable_a_throttle_plf();
  for (int i = 0; i < in_output_proto.a_throttle_plf().x_size(); i++) {
    const double x_out = in_output_proto.a_throttle_plf().x().Get(i);
    a_throttle_proto->add_x(x_out);
    a_throttle_proto->add_y(throttle_polyfit_.GetPolyVal(x_out));
  }

  sample_proto_.mutable_a_throttle_plf()->CopyFrom(out_sample.a_throttle_plf());
  output_proto_.mutable_a_throttle_plf()->CopyFrom(out_proto.a_throttle_plf());
}

void CalibrationTools::UpdateBrakeByFilter(
    const CalibrationProto& in_sample_proto,
    const CalibrationProto& in_output_proto) {
  const auto a_brake_plf =
      PiecewiseLinearFunctionFromProto(in_output_proto.a_brake_plf());

  CalibrationProto out_sample;
  auto* a_brake_sample = out_sample.mutable_a_brake_plf();
  std::vector<double> x, y;
  constexpr double kBrakeLimit = 5.0;
  for (int i = 0; i < in_sample_proto.a_brake_plf().x_size(); ++i) {
    const double y_sample = in_sample_proto.a_brake_plf().y().Get(i);
    const double y_evaluate =
        a_brake_plf.Evaluate(in_sample_proto.a_brake_plf().x().Get(i));
    if (std::fabs(y_sample - y_evaluate) < kBrakeLimit) {
      x.push_back(in_sample_proto.a_brake_plf().x().Get(i));
      y.push_back(in_sample_proto.a_brake_plf().y().Get(i));
      a_brake_sample->add_x(x.back());
      a_brake_sample->add_y(y.back());
    }
  }
  if (!brake_polyfit_.ComputerCoff(x, y)) return;

  CalibrationProto out_proto;
  auto* a_brake_proto = out_proto.mutable_a_brake_plf();
  for (int i = 0; i < in_output_proto.a_brake_plf().x_size(); i++) {
    const double x_out = in_output_proto.a_brake_plf().x().Get(i);
    a_brake_proto->add_x(x_out);
    a_brake_proto->add_y(brake_polyfit_.GetPolyVal(x_out));
  }

  sample_proto_.mutable_a_brake_plf()->CopyFrom(out_sample.a_brake_plf());
  output_proto_.mutable_a_brake_plf()->CopyFrom(out_proto.a_brake_plf());
}

}  // namespace calibration
}  // namespace control
}  // namespace qcraft
