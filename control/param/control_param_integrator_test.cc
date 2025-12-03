#include "onboard/control/param/control_param_integrator.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"

namespace qcraft::control {
namespace {

TEST(IntegrateControlParamTest, IntegrateTsPkmpcControllerConfTest) {
  ControllerConf controller_conf;
  EXPECT_OK(IntegrateControlParam(VEHICLE_LINCOLN_MKZ, &controller_conf));
  EXPECT_TRUE(controller_conf.has_ts_pkmpc_controller_conf());
  EXPECT_TRUE(controller_conf.has_lon_ts_pkmpc_controller_conf());
  EXPECT_EQ(controller_conf.lon_ts_pkmpc_controller_conf().matrix_q_size(), 3);
  EXPECT_GT(controller_conf.lon_ts_pkmpc_controller_conf().ts(), 0.0);
}

}  // namespace
}  // namespace qcraft::control
