#include "onboard/planner/optimization/problem/third_order_bicycle.h"

#include "gflags/gflags.h"

#include "gtest/gtest.h"

#include "onboard/math/convergence_order.h"
#include "onboard/planner/test_util/util.h"

DEFINE_bool(
    ddp_tob_use_f_second_derivative, false,
    "Whether to include second derivatives of f (ddfdxdx, ddfdudx, ddfdudu)");

namespace qcraft {
namespace planner {
namespace {

constexpr int kTrajectorySteps = 100;
constexpr double kTrajectoryTimeStep = 0.1;

using Tob = ThirdOrderBicycle;

constexpr double kEps = 1e-10;

MotionConstraintParamsProto motion_constraint_params =
    DefaultPlannerParams().motion_constraint_params();
VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();
VehicleDriveParamsProto veh_drive_params = DefaultVehicleDriveParams();

TEST(ThirdOrderBicycleTest, FTest_FreeRun) {
  Tob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
          kTrajectorySteps, kTrajectoryTimeStep,
          /*enable_dynamic_2nd_derivatives=*/false,
          /*enable_post_process=*/false);
  for (double v = 0.0; v < 1.1; v += 0.5) {
    const Tob::StateType x = Tob::MakeState(0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0);
    const Tob::ControlType u = {0.0, 0.0};
    const Tob::StateType x_next = ddp.EvaluateF(0, x, u);
    EXPECT_NEAR(x_next[0], v * kTrajectoryTimeStep, kEps);
    EXPECT_NEAR(x_next[1], 0.0, kEps);
    EXPECT_NEAR(x_next[2], 0.0, kEps);
    EXPECT_NEAR(x_next[3], v, kEps);
    EXPECT_NEAR(x_next[6], v * kTrajectoryTimeStep, kEps);
  }

  for (double v = 0.0; v < 1.1; v += 0.5) {
    const Tob::StateType x =
        Tob::MakeState(0.0, 0.0, M_PI * 0.5, v, 0.0, 0.0, 0.0);
    const Tob::ControlType u = {0.0, 0.0};
    const Tob::StateType x_next = ddp.EvaluateF(0, x, u);
    EXPECT_NEAR(x_next[0], 0.0, kEps);
    EXPECT_NEAR(x_next[1], v * kTrajectoryTimeStep, kEps);
    EXPECT_NEAR(x_next[2], M_PI * 0.5, kEps);
    EXPECT_NEAR(x_next[3], v, kEps);
    EXPECT_NEAR(x_next[6], v * kTrajectoryTimeStep, kEps);
  }

  for (double theta = 0.0; theta < M_PI * 1.1; theta += M_PI * 0.25) {
    const Tob::StateType x =
        Tob::MakeState(0.0, 0.0, theta, 1.0, 0.0, 0.0, 0.0);
    const Tob::ControlType u = {0.0, 0.0};
    const Tob::StateType x_next = ddp.EvaluateF(0, x, u);
    EXPECT_NEAR(x_next[0], std::cos(theta) * kTrajectoryTimeStep, kEps);
    EXPECT_NEAR(x_next[1], std::sin(theta) * kTrajectoryTimeStep, kEps);
    EXPECT_NEAR(x_next[2], theta, kEps);
    EXPECT_NEAR(x_next[3], 1.0, kEps);
    EXPECT_NEAR(x_next[6], kTrajectoryTimeStep, kEps);
  }
}

TEST(ThirdOrderBicycleTest, DFDxTest) {
  Tob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
          kTrajectorySteps, kTrajectoryTimeStep,
          /*enable_dynamic_2nd_derivatives=*/false,
          /*enable_post_process=*/false);
  const std::vector<Tob::StateType> xs = {
      Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
      Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
  const std::vector<Tob::ControlType> us = {
      Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
      Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};
  for (const Tob::StateType& x : xs) {
    for (const Tob::ControlType& u : us) {
      const Tob::StateType x_next = ddp.EvaluateF(0, x, u);
      LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                << " x_next = " << x_next.transpose();

      Tob::FDerivatives fd;
      ddp.EvaluateFDerivatives(0, x, u, &fd);
      const Tob::DFDxType& dfdx = fd.dfdx;

      for (int i = 0; i < Tob::kStateSize; ++i) {
        const int order = AssessConvergenceOrder([&](double perturbation) {
          Tob::StateType dx = Tob::StateType::Zero();
          dx[i] = perturbation;
          const Tob::StateType x_next_diff =
              ddp.EvaluateF(0, x + dx, u) - x_next;
          const Tob::StateType x_next_der = dfdx * dx;
          return (x_next_diff - x_next_der).norm();
        });
        LOG(INFO) << "i = " << i << " order = " << order;
        if (order != -1) EXPECT_EQ(order, 2);
      }
    }
  }
}

TEST(ThirdOrderBicycleTest, DFDuTest) {
  Tob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
          kTrajectorySteps, kTrajectoryTimeStep,
          /*enable_dynamic_2nd_derivatives=*/false,
          /*enable_post_process=*/false);
  const std::vector<Tob::StateType> xs = {
      Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
      Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
  const std::vector<Tob::ControlType> us = {
      Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
      Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};
  for (const Tob::StateType& x : xs) {
    for (const Tob::ControlType& u : us) {
      const Tob::StateType x_next = ddp.EvaluateF(0, x, u);
      LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                << " x_next = " << x_next.transpose();

      Tob::FDerivatives fd;
      ddp.EvaluateFDerivatives(0, x, u, &fd);
      const Tob::DFDuType& dfdu = fd.dfdu;

      for (int i = 0; i < Tob::kControlSize; ++i) {
        const int order = AssessConvergenceOrder([&](double perturbation) {
          Tob::ControlType du = Tob::ControlType::Zero();
          du[i] = perturbation;
          const Tob::StateType x_next_diff =
              ddp.EvaluateF(0, x, u + du) - x_next;
          const Tob::StateType x_next_der = dfdu * du;
          return (x_next_diff - x_next_der).norm();
        });
        LOG(INFO) << "i = " << i << " order = " << order;
        if (order != -1) EXPECT_EQ(order, 2);
      }
    }
  }
}

TEST(ThirdOrderBicycleTest, DDFDxDxTest) {
  Tob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
          kTrajectorySteps, kTrajectoryTimeStep,
          /*enable_dynamic_2nd_derivatives=*/false,
          /*enable_post_process=*/false);
  const std::vector<Tob::StateType> xs = {
      Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
      Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
  const std::vector<Tob::ControlType> us = {
      Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
      Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};
  for (const Tob::StateType& x : xs) {
    for (const Tob::ControlType& u : us) {
      const Tob::StateType x_next = ddp.EvaluateF(0, x, u);
      LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                << " x_next = " << x_next.transpose();

      Tob::FDerivatives fd;
      ddp.EvaluateFDerivatives(0, x, u, &fd);
      const Tob::DDFDxDxType& ddfdxdx = fd.ddfdxdx;

      for (int i = 0; i < Tob::kStateSize; ++i) {
        for (int j = 0; j < Tob::kStateSize; ++j) {
          const int order = AssessConvergenceOrder([&](double perturbation) {
            Tob::StateType dxi = Tob::StateType::Zero();
            dxi[i] = perturbation;
            Tob::StateType dxj = Tob::StateType::Zero();
            dxj[j] = perturbation;
            const Tob::StateType x_next_diff =
                ddp.EvaluateF(0, x + dxj + dxi, u) + x_next -
                ddp.EvaluateF(0, x + dxj, u) - ddp.EvaluateF(0, x + dxi, u);
            Tob::StateType x_next_der;
            for (int k = 0; k < Tob::kStateSize; ++k) {
              x_next_der[k] = dxi.transpose() * ddfdxdx[k] * dxj;
            }
            return (x_next_diff - x_next_der).norm();
          });
          LOG(INFO) << "i = " << i << " j = " << j << " order = " << order;
          if (order != -1)
            EXPECT_EQ(order, FLAGS_ddp_tob_use_f_second_derivative ? 3 : 2);
        }
      }
    }
  }
}

TEST(ThirdOrderBicycleTest, DDFDuDxTest) {
  Tob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
          kTrajectorySteps, kTrajectoryTimeStep,
          /*enable_dynamic_2nd_derivatives=*/false,
          /*enable_post_process=*/false);
  const std::vector<Tob::StateType> xs = {
      Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
      Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
  const std::vector<Tob::ControlType> us = {
      Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
      Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};
  for (const Tob::StateType& x : xs) {
    for (const Tob::ControlType& u : us) {
      const Tob::StateType x_next = ddp.EvaluateF(0, x, u);
      LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                << " x_next = " << x_next.transpose();

      Tob::FDerivatives fd;
      ddp.EvaluateFDerivatives(0, x, u, &fd);
      const Tob::DDFDuDxType& ddfdudx = fd.ddfdudx;

      for (int i = 0; i < Tob::kControlSize; ++i) {
        for (int j = 0; j < Tob::kStateSize; ++j) {
          const int order = AssessConvergenceOrder([&](double perturbation) {
            Tob::ControlType dui = Tob::ControlType::Zero();
            dui[i] = perturbation;
            Tob::StateType dxj = Tob::StateType::Zero();
            dxj[j] = perturbation;
            const Tob::StateType x_next_diff =
                ddp.EvaluateF(0, x + dxj, u + dui) + x_next -
                ddp.EvaluateF(0, x + dxj, u) - ddp.EvaluateF(0, x, u + dui);
            Tob::StateType x_next_der;
            for (int k = 0; k < Tob::kStateSize; ++k) {
              x_next_der[k] = dui.transpose() * ddfdudx[k] * dxj;
            }
            return (x_next_diff - x_next_der).norm();
          });
          LOG(INFO) << "i = " << i << " j = " << j << " order = " << order;
          if (order != -1)
            EXPECT_EQ(order, FLAGS_ddp_tob_use_f_second_derivative ? 3 : 2);
        }
      }
    }
  }
}

TEST(ThirdOrderBicycleTest, DDFDuDuTest) {
  Tob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
          kTrajectorySteps, kTrajectoryTimeStep,
          /*enable_dynamic_2nd_derivatives=*/false,
          /*enable_post_process=*/false);
  const std::vector<Tob::StateType> xs = {
      Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
      Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
  const std::vector<Tob::ControlType> us = {
      Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
      Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};
  for (const Tob::StateType& x : xs) {
    for (const Tob::ControlType& u : us) {
      const Tob::StateType x_next = ddp.EvaluateF(0, x, u);
      LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                << " x_next = " << x_next.transpose();

      Tob::FDerivatives fd;
      ddp.EvaluateFDerivatives(0, x, u, &fd);
      const Tob::DDFDuDuType& ddfdudu = fd.ddfdudu;

      for (int i = 0; i < Tob::kControlSize; ++i) {
        for (int j = 0; j < Tob::kControlSize; ++j) {
          const int order = AssessConvergenceOrder([&](double perturbation) {
            Tob::ControlType dui = Tob::ControlType::Zero();
            dui[i] = perturbation;
            Tob::ControlType duj = Tob::ControlType::Zero();
            duj[j] = perturbation;
            const Tob::StateType x_next_diff =
                ddp.EvaluateF(0, x, u + duj + dui) + x_next -
                ddp.EvaluateF(0, x, u + duj) - ddp.EvaluateF(0, x, u + dui);
            Tob::StateType x_next_der;
            for (int k = 0; k < Tob::kStateSize; ++k) {
              x_next_der[k] = dui.transpose() * ddfdudu[k] * duj;
            }
            return (x_next_diff - x_next_der).norm();
          });
          LOG(INFO) << "i = " << i << " j = " << j << " order = " << order;
          if (order != -1)
            EXPECT_EQ(order, FLAGS_ddp_tob_use_f_second_derivative ? 3 : 2);
        }
      }
    }
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
