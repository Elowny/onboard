#include "onboard/planner/util/motion_util.h"

#include <algorithm>
#include <memory>

#include "gtest/gtest.h"

namespace qcraft {
namespace planner {

namespace {
void TestConstJerkBrakingMotion(double v0, double a0, double min_acc,
                                double jerk, double t_max) {
  constexpr double kBruteForceTStep = 0.001;
  constexpr double kError = 0.1;

  ConstJerkBrakingMotion motion(v0, a0, min_acc, jerk);

  a0 = std::max(a0, min_acc);
  int num_steps = static_cast<int>(t_max / kBruteForceTStep) + 1;
  const double t_step = t_max / num_steps;

  double s = 0.0;
  double v = v0;
  double a = a0;

  for (int i = 0; i < num_steps; ++i) {
    const double new_t = (i + 1) * t_step;
    const double new_s = s + v * t_step;
    const double new_v = std::max(v + a * t_step, 0.0);
    const double new_a = std::max(a + jerk * t_step, min_acc);
    s = new_s;
    v = new_v;
    a = new_a;

    const double s_analytical = motion.GetS(new_t);
    const double v_analytical = motion.GetV(new_t);
    const double a_analytical = motion.GetA(new_t);

    EXPECT_NEAR(s, s_analytical, kError);
    EXPECT_NEAR(v, v_analytical, kError);
    EXPECT_NEAR(a, a_analytical, kError);
  }
}

TEST(PlannerUtilTest, ConstJerkBrakingMotionTest) {
  {
    const double v0 = 10.0;
    const double a0 = 1.0;
    const double jerk = -4.0;
    const double min_acc = -3.0;
    const double t_max = 5.0;
    TestConstJerkBrakingMotion(v0, a0, min_acc, jerk, t_max);
  }

  {
    const double v0 = 10.0;
    const double a0 = -6.0;
    const double jerk = -2.0;
    const double min_acc = -3.0;
    const double t_max = 5.0;
    TestConstJerkBrakingMotion(v0, a0, min_acc, jerk, t_max);
  }

  {
    const double v0 = 3.0;
    const double a0 = -3.0;
    const double jerk = -0.5;
    const double min_acc = -8.0;
    const double t_max = 2.0;
    TestConstJerkBrakingMotion(v0, a0, min_acc, jerk, t_max);
  }

  {
    const double v0 = 15.0;
    const double a0 = 3.0;
    const double jerk = -8.0;
    const double min_acc = -8.0;
    const double t_max = 5.0;
    TestConstJerkBrakingMotion(v0, a0, min_acc, jerk, t_max);
  }
}

void TestConstJerkAcceleratingMotion(double v0, double a0, double max_acc,
                                     double jerk, double t_max) {
  constexpr double kBruteForceTStep = 0.001;
  constexpr double kError = 0.1;

  ConstJerkAcceleratingMotion motion(v0, a0, max_acc, jerk);

  a0 = std::min(a0, max_acc);
  int num_steps = static_cast<int>(t_max / kBruteForceTStep) + 1;
  const double t_step = t_max / num_steps;

  double s = 0.0;
  double v = v0;
  double a = a0;

  for (int i = 0; i < num_steps; ++i) {
    const double new_t = (i + 1) * t_step;
    const double new_s = s + v * t_step;
    const double new_v = std::max(v + a * t_step, 0.0);
    const double new_a = std::min(a + jerk * t_step, max_acc);
    s = new_s;
    v = new_v;
    a = new_a;

    const double s_analytical = motion.GetS(new_t);
    const double v_analytical = motion.GetV(new_t);
    const double a_analytical = motion.GetA(new_t);

    EXPECT_NEAR(s, s_analytical, kError);
    EXPECT_NEAR(v, v_analytical, kError);
    EXPECT_NEAR(a, a_analytical, kError);
  }
}

TEST(PlannerUtilTest, ConstJerkAcceleratingMotionTest) {
  {
    const double v0 = 10.0;
    const double a0 = 1.0;
    const double jerk = 4.0;
    const double max_acc = 3.0;
    const double t_max = 5.0;
    TestConstJerkAcceleratingMotion(v0, a0, max_acc, jerk, t_max);
  }

  {
    const double v0 = 10.0;
    const double a0 = -6.0;
    const double jerk = 2.0;
    const double max_acc = 3.0;
    const double t_max = 5.0;
    TestConstJerkAcceleratingMotion(v0, a0, max_acc, jerk, t_max);
  }

  {
    const double v0 = 3.0;
    const double a0 = -3.0;
    const double jerk = 0.5;
    const double max_acc = 1.0;
    const double t_max = 2.0;
    TestConstJerkAcceleratingMotion(v0, a0, max_acc, jerk, t_max);
  }

  {
    const double v0 = 3.0;
    const double a0 = -5.0;
    const double jerk = 1.0;
    const double max_acc = 1.0;
    const double t_max = 8.0;
    TestConstJerkAcceleratingMotion(v0, a0, max_acc, jerk, t_max);
  }
}

void TestConstJerkMotion(double v0, double a0, double min_acc, double max_acc,
                         double jerk, double t_max) {
  constexpr double kBruteForceTStep = 0.001;
  constexpr double kError = 0.1;

  ConstJerkMotion motion(v0, a0, min_acc, max_acc, jerk);
  a0 = std::clamp(a0, min_acc, max_acc);

  int num_steps = static_cast<int>(t_max / kBruteForceTStep) + 1;
  const double t_step = t_max / num_steps;

  double s = 0.0;
  double v = v0;
  double a = a0;

  for (int i = 0; i < num_steps; ++i) {
    const double new_t = (i + 1) * t_step;
    const double new_s = s + v * t_step;
    const double new_v = std::max(v + a * t_step, 0.0);
    const double new_a = std::clamp(a + jerk * t_step, min_acc, max_acc);
    s = new_s;
    v = new_v;
    a = new_a;

    const double s_analytical = motion.GetS(new_t);
    const double v_analytical = motion.GetV(new_t);
    const double a_analytical = motion.GetA(new_t);

    EXPECT_NEAR(s, s_analytical, kError);
    EXPECT_NEAR(v, v_analytical, kError);
    EXPECT_NEAR(a, a_analytical, kError);
  }
}

TEST(PlannerUtilTest, ConstJerkMotionTest) {
  {
    const double v0 = 10.0;
    const double a0 = 1.0;
    const double jerk = 4.0;
    const double min_acc = -8.0;
    const double max_acc = 3.0;
    const double t_max = 5.0;
    TestConstJerkMotion(v0, a0, min_acc, max_acc, jerk, t_max);
  }

  {
    const double v0 = 10.0;
    const double a0 = -6.0;
    const double jerk = 2.0;
    const double min_acc = -8.0;
    const double max_acc = 3.0;
    const double t_max = 5.0;
    TestConstJerkMotion(v0, a0, min_acc, max_acc, jerk, t_max);
  }

  {
    const double v0 = 3.0;
    const double a0 = -3.0;
    const double jerk = 0.5;
    const double min_acc = -8.0;
    const double max_acc = 1.0;
    const double t_max = 2.0;
    TestConstJerkMotion(v0, a0, min_acc, max_acc, jerk, t_max);
  }

  {
    const double v0 = 10.0;
    const double a0 = 1.0;
    const double jerk = -4.0;
    const double min_acc = -3.0;
    const double max_acc = 1.0;
    const double t_max = 5.0;
    TestConstJerkMotion(v0, a0, min_acc, max_acc, jerk, t_max);
  }

  {
    const double v0 = 10.0;
    const double a0 = -6.0;
    const double jerk = -2.0;
    const double min_acc = -3.0;
    const double max_acc = 1.0;
    const double t_max = 5.0;
    TestConstJerkMotion(v0, a0, min_acc, max_acc, jerk, t_max);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
