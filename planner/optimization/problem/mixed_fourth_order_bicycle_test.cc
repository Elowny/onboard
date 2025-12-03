#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"

#include "gflags/gflags.h"

#include "gtest/gtest.h"

#include "onboard/math/convergence_order.h"
#include "onboard/planner/test_util/util.h"

DEFINE_bool(
    ddp_mfob_use_f_second_derivative, false,
    "Whether to include second derivatives of f (ddfdxdx, ddfdudx, ddfdudu)");

namespace qcraft {
namespace planner {
namespace {

constexpr int kTrajectorySteps = 100;
constexpr double kTrajectoryTimeStep = 0.1;

using Mfob = MixedFourthOrderBicycle;

constexpr double kEps = 1e-10;

MotionConstraintParamsProto motion_constraint_params =
    DefaultPlannerParams().motion_constraint_params();
VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();
VehicleDriveParamsProto veh_drive_params = DefaultVehicleDriveParams();

TEST(MixedFourthOrderBicycleTest, FTest_FreeRun) {
  Mfob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
           kTrajectorySteps, kTrajectoryTimeStep,
           /*enable_dynamic_2nd_derivatives=*/false,
           /*enable_post_process=*/false);
  for (double v = 0.0; v < 1.1; v += 0.5) {
    const Mfob::StateType x =
        Mfob::MakeState(0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, 0.0);
    const Mfob::ControlType u = {0.0, 0.0};
    const Mfob::StateType x_next = ddp.EvaluateF(0, x, u);
    EXPECT_NEAR(x_next[0], v * ddp.dt(), kEps);
    EXPECT_NEAR(x_next[1], 0.0, kEps);
    EXPECT_NEAR(x_next[2], 0.0, kEps);
    EXPECT_NEAR(x_next[3], v, kEps);
    EXPECT_NEAR(x_next[7], v * ddp.dt(), kEps);
  }

  for (double v = 0.0; v < 1.1; v += 0.5) {
    const Mfob::StateType x =
        Mfob::MakeState(0.0, 0.0, M_PI * 0.5, v, 0.0, 0.0, 0.0, 0.0);
    const Mfob::ControlType u = {0.0, 0.0};
    const Mfob::StateType x_next = ddp.EvaluateF(0, x, u);
    EXPECT_NEAR(x_next[0], 0.0, kEps);
    EXPECT_NEAR(x_next[1], v * ddp.dt(), kEps);
    EXPECT_NEAR(x_next[2], M_PI * 0.5, kEps);
    EXPECT_NEAR(x_next[3], v, kEps);
    EXPECT_NEAR(x_next[7], v * ddp.dt(), kEps);
  }

  for (double theta = 0.0; theta < M_PI * 1.1; theta += M_PI * 0.25) {
    const Mfob::StateType x =
        Mfob::MakeState(0.0, 0.0, theta, 1.0, 0.0, 0.0, 0.0, 0.0);
    const Mfob::ControlType u = {0.0, 0.0};
    const Mfob::StateType x_next = ddp.EvaluateF(0, x, u);
    EXPECT_NEAR(x_next[0], std::cos(theta) * ddp.dt(), kEps);
    EXPECT_NEAR(x_next[1], std::sin(theta) * ddp.dt(), kEps);
    EXPECT_NEAR(x_next[2], theta, kEps);
    EXPECT_NEAR(x_next[3], 1.0, kEps);
    EXPECT_NEAR(x_next[7], ddp.dt(), kEps);
  }
}

TEST(MixedFourthOrderBicycleTest, DFDxTest) {
  Mfob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
           kTrajectorySteps, kTrajectoryTimeStep,
           /*enable_dynamic_2nd_derivatives=*/false,
           /*enable_post_process=*/false);
  const std::vector<Mfob::StateType> xs = {
      Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
      Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4)};
  const std::vector<Mfob::ControlType> us = {
      Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
      Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};
  for (const Mfob::StateType& x : xs) {
    for (const Mfob::ControlType& u : us) {
      const Mfob::StateType x_next = ddp.EvaluateF(0, x, u);
      LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                << " x_next = " << x_next.transpose();

      Mfob::FDerivatives fd;
      ddp.EvaluateFDerivatives(0, x, u, &fd);
      const Mfob::DFDxType& dfdx = fd.dfdx;

      for (int i = 0; i < Mfob::kStateSize; ++i) {
        const int order = AssessConvergenceOrder([&](double perturbation) {
          Mfob::StateType dx = Mfob::StateType::Zero();
          dx[i] = perturbation;
          const Mfob::StateType x_next_diff =
              ddp.EvaluateF(0, x + dx, u) - x_next;
          const Mfob::StateType x_next_der = dfdx * dx;
          return (x_next_diff - x_next_der).norm();
        });
        LOG(INFO) << "i = " << i << " order = " << order;
        if (order != -1) EXPECT_EQ(order, 2);
      }
    }
  }
}

TEST(MixedFourthOrderBicycleTest, DFDuTest) {
  Mfob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
           kTrajectorySteps, kTrajectoryTimeStep,
           /*enable_dynamic_2nd_derivatives=*/false,
           /*enable_post_process=*/false);
  const std::vector<Mfob::StateType> xs = {
      Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
      Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4)};
  const std::vector<Mfob::ControlType> us = {
      Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
      Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};
  for (const Mfob::StateType& x : xs) {
    for (const Mfob::ControlType& u : us) {
      const Mfob::StateType x_next = ddp.EvaluateF(0, x, u);
      LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                << " x_next = " << x_next.transpose();

      Mfob::FDerivatives fd;
      ddp.EvaluateFDerivatives(0, x, u, &fd);
      const Mfob::DFDuType& dfdu = fd.dfdu;

      for (int i = 0; i < Mfob::kControlSize; ++i) {
        const int order = AssessConvergenceOrder([&](double perturbation) {
          Mfob::ControlType du = Mfob::ControlType::Zero();
          du[i] = perturbation;
          const Mfob::StateType x_next_diff =
              ddp.EvaluateF(0, x, u + du) - x_next;
          const Mfob::StateType x_next_der = dfdu * du;
          return (x_next_diff - x_next_der).norm();
        });
        LOG(INFO) << "i = " << i << " order = " << order;
        if (order != -1) EXPECT_EQ(order, 2);
      }
    }
  }
}

TEST(MixedFourthOrderBicycleTest, DDFDxDxTest) {
  Mfob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
           kTrajectorySteps, kTrajectoryTimeStep,
           /*enable_dynamic_2nd_derivatives=*/false,
           /*enable_post_process=*/false);
  const std::vector<Mfob::StateType> xs = {
      Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
      Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4)};
  const std::vector<Mfob::ControlType> us = {
      Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
      Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};
  for (const Mfob::StateType& x : xs) {
    for (const Mfob::ControlType& u : us) {
      const Mfob::StateType x_next = ddp.EvaluateF(0, x, u);
      LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                << " x_next = " << x_next.transpose();

      Mfob::FDerivatives fd;
      ddp.EvaluateFDerivatives(0, x, u, &fd);
      const Mfob::DDFDxDxType& ddfdxdx = fd.ddfdxdx;

      for (int i = 0; i < Mfob::kStateSize; ++i) {
        for (int j = 0; j < Mfob::kStateSize; ++j) {
          const int order = AssessConvergenceOrder([&](double perturbation) {
            Mfob::StateType dxi = Mfob::StateType::Zero();
            dxi[i] = perturbation;
            Mfob::StateType dxj = Mfob::StateType::Zero();
            dxj[j] = perturbation;
            const Mfob::StateType x_next_diff =
                ddp.EvaluateF(0, x + dxj + dxi, u) + x_next -
                ddp.EvaluateF(0, x + dxj, u) - ddp.EvaluateF(0, x + dxi, u);
            Mfob::StateType x_next_der;
            for (int k = 0; k < Mfob::kStateSize; ++k) {
              x_next_der[k] = dxi.transpose() * ddfdxdx[k] * dxj;
            }
            return (x_next_diff - x_next_der).norm();
          });
          LOG(INFO) << "i = " << i << " j = " << j << " order = " << order;
          if (order != -1)
            EXPECT_EQ(order, FLAGS_ddp_mfob_use_f_second_derivative ? 3 : 2);
        }
      }
    }
  }
}

TEST(MixedFourthOrderBicycleTest, DDFDuDxTest) {
  Mfob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
           kTrajectorySteps, kTrajectoryTimeStep,
           /*enable_dynamic_2nd_derivatives=*/false,
           /*enable_post_process=*/false);
  const std::vector<Mfob::StateType> xs = {
      Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
      Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4)};
  const std::vector<Mfob::ControlType> us = {
      Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
      Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};
  for (const Mfob::StateType& x : xs) {
    for (const Mfob::ControlType& u : us) {
      const Mfob::StateType x_next = ddp.EvaluateF(0, x, u);
      LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                << " x_next = " << x_next.transpose();

      Mfob::FDerivatives fd;
      ddp.EvaluateFDerivatives(0, x, u, &fd);
      const Mfob::DDFDuDxType& ddfdudx = fd.ddfdudx;

      for (int i = 0; i < Mfob::kControlSize; ++i) {
        for (int j = 0; j < Mfob::kStateSize; ++j) {
          const int order = AssessConvergenceOrder([&](double perturbation) {
            Mfob::ControlType dui = Mfob::ControlType::Zero();
            dui[i] = perturbation;
            Mfob::StateType dxj = Mfob::StateType::Zero();
            dxj[j] = perturbation;
            const Mfob::StateType x_next_diff =
                ddp.EvaluateF(0, x + dxj, u + dui) + x_next -
                ddp.EvaluateF(0, x + dxj, u) - ddp.EvaluateF(0, x, u + dui);
            Mfob::StateType x_next_der;
            for (int k = 0; k < Mfob::kStateSize; ++k) {
              x_next_der[k] = dui.transpose() * ddfdudx[k] * dxj;
            }
            return (x_next_diff - x_next_der).norm();
          });
          LOG(INFO) << "i = " << i << " j = " << j << " order = " << order;
          if (order != -1)
            EXPECT_EQ(order, FLAGS_ddp_mfob_use_f_second_derivative ? 3 : 2);
        }
      }
    }
  }
}

TEST(MixedFourthOrderBicycleTest, DDFDuDuTest) {
  Mfob ddp(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
           kTrajectorySteps, kTrajectoryTimeStep,
           /*enable_dynamic_2nd_derivatives=*/false,
           /*enable_post_process=*/false);
  const std::vector<Mfob::StateType> xs = {
      Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
      Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4)};
  const std::vector<Mfob::ControlType> us = {
      Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
      Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};
  for (const Mfob::StateType& x : xs) {
    for (const Mfob::ControlType& u : us) {
      const Mfob::StateType x_next = ddp.EvaluateF(0, x, u);
      LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                << " x_next = " << x_next.transpose();

      Mfob::FDerivatives fd;
      ddp.EvaluateFDerivatives(0, x, u, &fd);
      const Mfob::DDFDuDuType& ddfdudu = fd.ddfdudu;

      for (int i = 0; i < Mfob::kControlSize; ++i) {
        for (int j = 0; j < Mfob::kControlSize; ++j) {
          const int order = AssessConvergenceOrder([&](double perturbation) {
            Mfob::ControlType dui = Mfob::ControlType::Zero();
            dui[i] = perturbation;
            Mfob::ControlType duj = Mfob::ControlType::Zero();
            duj[j] = perturbation;
            const Mfob::StateType x_next_diff =
                ddp.EvaluateF(0, x, u + duj + dui) + x_next -
                ddp.EvaluateF(0, x, u + duj) - ddp.EvaluateF(0, x, u + dui);
            Mfob::StateType x_next_der;
            for (int k = 0; k < Mfob::kStateSize; ++k) {
              x_next_der[k] = dui.transpose() * ddfdudu[k] * duj;
            }
            return (x_next_diff - x_next_der).norm();
          });
          LOG(INFO) << "i = " << i << " j = " << j << " order = " << order;
          if (order != -1)
            EXPECT_EQ(order, FLAGS_ddp_mfob_use_f_second_derivative ? 3 : 2);
        }
      }
    }
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
