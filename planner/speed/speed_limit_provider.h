#ifndef ONBOARD_PLANNER_SPEED_SPEED_LIMIT_PROVIDER_H_
#define ONBOARD_PLANNER_SPEED_SPEED_LIMIT_PROVIDER_H_

#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/vt_speed_limit.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner {

// This class provided query interfaces for dynamic speed limit (time-vary speed
// limit) and static speed limit (time-invariant speed limit) or both.
class SpeedLimitProvider {
 public:
  using SpeedLimitInfo = SpeedLimit::SpeedLimitInfo;

  SpeedLimitProvider(
      std::map<SpeedLimitTypeProto::Type, SpeedLimit> static_speed_limit_map,
      std::vector<std::optional<SpeedLimit>> dynamic_speed_limit,
      std::map<SpeedLimitTypeProto::Type, VtSpeedLimit> vt_speed_limit_map,
      double time_step)
      : static_speed_limit_map_(std::move(static_speed_limit_map)),
        dynamic_speed_limit_(std::move(dynamic_speed_limit)),
        vt_speed_limit_map_(std::move(vt_speed_limit_map)),
        time_step_(time_step) {
    QCHECK_GT(time_step_, 0.0);
    GenerateMinVtSpeedLimit();
  }

  // Joint query for dynamic speed limit, combination static speed limit and v-t
  // speed limit.
  std::optional<SpeedLimitInfo> GetSpeedLimitInfoByTimeAndS(double t,
                                                            double s) const;
  std::optional<double> GetSpeedLimitByTimeAndS(double t, double s) const;

  // Query combination static speed limit.
  std::optional<SpeedLimitInfo> GetStaticSpeedLimitInfoByS(double s) const;
  std::optional<double> GetStaticSpeedLimitByS(double s) const;

  // Query dynamic speed limit.
  std::optional<SpeedLimitInfo> GetDynamicSpeedLimitInfoByTimeAndS(
      double t, double s) const;
  const SpeedLimit& GetCombinationStaticSpeedLimit() const {
    return FindOrDie(static_speed_limit_map_, SpeedLimitTypeProto::COMBINATION);
  }

  // Query vt speed limit.
  std::optional<SpeedLimit::SpeedLimitInfo> GetVtSpeedLimitInfoByTime(
      double t) const;
  std::optional<SpeedLimit::SpeedLimitInfo> GetVtSpeedLimitInfoByTypeAndTime(
      const SpeedLimitTypeProto::Type type, double t) const;

  void AddVtSpeedLimit(SpeedLimitTypeProto::Type key,
                       const VtSpeedLimit& value) {
    if (InsertOrUpdate(&vt_speed_limit_map_, key, value)) {
      UpdateMinVtSpeedLimit(value);
    } else {
      GenerateMinVtSpeedLimit();
    }
  }

  const std::map<SpeedLimitTypeProto::Type, SpeedLimit>&
  static_speed_limit_map() const {
    return static_speed_limit_map_;
  }

  const std::map<SpeedLimitTypeProto::Type, VtSpeedLimit>& vt_speed_limit_map()
      const {
    return vt_speed_limit_map_;
  }

  double time_step() const { return time_step_; }

 private:
  void GenerateMinVtSpeedLimit();
  void UpdateMinVtSpeedLimit(const VtSpeedLimit& vt_speed_limit);

  std::map<SpeedLimitTypeProto::Type, SpeedLimit> static_speed_limit_map_;
  std::vector<std::optional<SpeedLimit>> dynamic_speed_limit_;
  std::map<SpeedLimitTypeProto::Type, VtSpeedLimit> vt_speed_limit_map_;
  VtSpeedLimit min_vt_speed_limit_;
  // The fixed time interval of dynamic speed limit and vt speed limit.
  double time_step_ = 0.0;
};

inline std::optional<double> SpeedLimitProvider::GetSpeedLimitByTimeAndS(
    double t, double s) const {
  const auto info = GetSpeedLimitInfoByTimeAndS(t, s);
  return info.has_value() ? std::make_optional<double>(info->speed_limit)
                          : std::nullopt;
}

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_LIMIT_PROVIDER_H_
