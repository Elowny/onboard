#include "onboard/control/control_util.h"

#include "onboard/params/v2/proto/vehicle/common.pb.h"

namespace qcraft::control {

bool HasPhysicalSteeringWheel(const VehicleModel& vehicle_model) {
  switch (vehicle_model) {
    case VEHICLE_LINCOLN_MKZ:
    case VEHICLE_LINCOLN_MKZ_AS_PACMOD:
    case VEHICLE_AION_LX:
    case VEHICLE_JINLV_MINIBUS:
    case VEHICLE_SKYWELL:
    case VEHICLE_HIGER:
    case VEHICLE_ZHONGTONG:
    case VEHICLE_BYD:
    case VEHICLE_TEST_BENCH:
    case VEHICLE_MARVELX:
    case VEHICLE_POLERSTAR:
    case VEHICLE_POLERSTAR_2:
    case VEHICLE_SHUNFENG:
    case VEHICLE_MARVELR:
    case VEHICLE_ZHONGTONG6:
    case VEHICLE_MARVELR_NEW:
    case VEHICLE_QCRAFTVEHICLE_SUV:
    case VEHICLE_HIGER65_LONGZHOU_ONE:
    case VEHICLE_HAVAL_SUV:
    case VEHICLE_HYUNDAI_CAR:
    case VEHICLE_SF5_SUV:
    case VEHICLE_AION_LX_PLUS_SUV:
    case VEHICLE_HYPER_AH8_SUV:
    case VEHICLE_HYPER_GT_CAR:
    case VEHICLE_FUKANG_CAR:
    case VEHICLE_ZEV_LONGZHOU_ONE:
    case VEHICLE_JETOUR_SUV:
    case VEHICLE_FOTON_LONGZHOU_LONG:
    case VEHICLE_HIGER85_LONGZHOU_LONG:
    case VEHICLE_JINLV85_LONGZHOU_LONG:
    case VEHICLE_SERES_SF5_SUV:
      return true;
    case VEHICLE_PIXLOOP:
    case VEHICLE_ZHONGTONG55:
    case VEHICLE_DONGFENG:
    case VEHICLE_ZHONGXING:
      return false;
    default:
      return true;
  }
}

}  // namespace qcraft::control
