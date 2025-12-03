#include "onboard/control/controllers/model/tob_tv_kinematic_model.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/eigen.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/util.h"

namespace qcraft::control {
namespace {

constexpr double kTs = 0.2;
constexpr int kHorizon = 10;
constexpr int kStateSize = 4;
constexpr int kInputSize = 1;
constexpr double kEpsilon = 0.05;

bool MatricsAreClosed(const Eigen::MatrixXd& matrix_a,
                      const Eigen::MatrixXd& matrix_b, double epsilon) {
  if (matrix_a.cols() != matrix_b.cols()) return false;
  if (matrix_a.rows() != matrix_b.rows()) return false;
  for (unsigned int i = 0; i < matrix_a.size(); ++i) {
    if (std::abs(matrix_a(i) - matrix_b(i)) > epsilon) return false;
  }
  return true;
}

TEST(TobTvKinematicModelUpdateTest, Test0) {
  std::vector<double> speed_vec;
  std::vector<double> yaw_ref;

  constexpr double kInitSpeed = 1.0;
  constexpr double kDeltaSpeed = 0.1;

  constexpr double kCurrYaw = 0.5;
  constexpr double kInitYawRef = 0.5;
  constexpr double kDeltaYawRef = 0.01;

  for (int i = 0; i < kHorizon; ++i) {
    speed_vec.push_back(kInitSpeed + kDeltaSpeed * kTs * i);
    yaw_ref.push_back(NormalizeAngle(kInitYawRef + kDeltaYawRef * kTs * i));
  }

  std::vector<Eigen::MatrixXd> expected_a = std::vector<Eigen::MatrixXd>(
      kHorizon, Eigen::MatrixXd::Zero(kStateSize, kStateSize));
  Eigen::MatrixXd expected_b = Eigen::Vector4d(0.0, 0.0, 0.0, kTs);
  std::vector<Eigen::MatrixXd> expected_w = std::vector<Eigen::MatrixXd>(
      kHorizon, Eigen::MatrixXd::Zero(kStateSize, kInputSize));
  for (int i = 0; i < kHorizon; ++i) {
    expected_a[i](0, 0) = 1.0;
    expected_a[i](1, 1) = 1.0;
    expected_a[i](2, 2) = 1.0;
    expected_a[i](3, 3) = 1.0;
    expected_a[i](0, 2) = -speed_vec[i] * std::sin(kCurrYaw) * kTs;
    expected_a[i](1, 2) = speed_vec[i] * std::cos(kCurrYaw) * kTs;
    expected_a[i](2, 3) = speed_vec[i] * kTs;

    const double cos_delta_yaw_ref = fast_math::Cos(yaw_ref[i] - yaw_ref[0]);
    expected_w[i] = Eigen::Vector4d(
        speed_vec[i] * std::cos(kCurrYaw) * cos_delta_yaw_ref * kTs,
        speed_vec[i] * std::sin(kCurrYaw) * cos_delta_yaw_ref * kTs, 0.0, 0.0);
  }

  // Function result;
  const auto function_result = TobTvKinematicModelUpdate(
      kTs, KinematicModelInput{
               .yaw = kCurrYaw, .speed = speed_vec, .ref_yaw = yaw_ref});

  for (int i = 0; i < kHorizon; ++i) {
    EXPECT_TRUE(MatricsAreClosed(function_result.state_space_vector[i].A,
                                 expected_a[i], kEpsilon))
        << "i = " << i << ", diff is:"
        << function_result.state_space_vector[i].A - expected_a[i];

    EXPECT_TRUE(MatricsAreClosed(function_result.state_space_vector[i].B,
                                 expected_b, kEpsilon))
        << "i = " << i
        << ", diff is:" << function_result.state_space_vector[i].B - expected_b;

    EXPECT_TRUE(MatricsAreClosed(function_result.state_space_vector[i].W,
                                 expected_w[i], kEpsilon))
        << "i = " << i << ", diff is:"
        << function_result.state_space_vector[i].W - expected_w[i];
  }
}
}  // namespace

}  // namespace qcraft::control
