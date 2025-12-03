#include "onboard/planner/speed/speed_limit_provider.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>

namespace qcraft::planner {
namespace {
using SpeedLimitInfo = SpeedLimit::SpeedLimitInfo;
}

void SpeedLimitProvider::GenerateMinVtSpeedLimit() {
  min_vt_speed_limit_.clear();
  for (const auto& [_, vt_speed_limit] : vt_speed_limit_map_) {
    UpdateMinVtSpeedLimit(vt_speed_limit);
  }
}

void SpeedLimitProvider::UpdateMinVtSpeedLimit(
    const VtSpeedLimit& vt_speed_limit) {
  if (vt_speed_limit.empty()) return;
  if (vt_speed_limit.size() > min_vt_speed_limit_.size()) {
    min_vt_speed_limit_.reserve(vt_speed_limit.size());
  }
  for (int i = 0; i < vt_speed_limit.size(); ++i) {
    if (i < min_vt_speed_limit_.size()) {
      if (vt_speed_limit[i].speed_limit < min_vt_speed_limit_[i].speed_limit) {
        min_vt_speed_limit_[i] = vt_speed_limit[i];
      }
    } else {
      min_vt_speed_limit_.push_back(vt_speed_limit[i]);
    }
  }
}

std::optional<SpeedLimitInfo> SpeedLimitProvider::GetSpeedLimitInfoByTimeAndS(
    double t, double s) const {
  const SpeedLimitInfo* speed_limit = nullptr;
  auto static_speed_limit_info = GetStaticSpeedLimitInfoByS(s);
  if (static_speed_limit_info.has_value()) {
    speed_limit = &*static_speed_limit_info;
  }
  auto dynamic_speed_limit_info = GetDynamicSpeedLimitInfoByTimeAndS(t, s);
  if (dynamic_speed_limit_info.has_value()) {
    if (speed_limit == nullptr ||
        dynamic_speed_limit_info->speed_limit < speed_limit->speed_limit) {
      speed_limit = &*dynamic_speed_limit_info;
    }
  }
  auto vt_speed_limit = GetVtSpeedLimitInfoByTime(t);
  if (vt_speed_limit.has_value()) {
    if (speed_limit == nullptr ||
        vt_speed_limit->speed_limit < speed_limit->speed_limit) {
      speed_limit = &*vt_speed_limit;
    }
  }
  if (speed_limit == nullptr) return std::nullopt;
  return *speed_limit;
}

// Only query combined static speed limit.
std::optional<SpeedLimitInfo> SpeedLimitProvider::GetStaticSpeedLimitInfoByS(
    double s) const {
  const auto& combine_speed_limit =
      FindOrDie(static_speed_limit_map_, SpeedLimitTypeProto::COMBINATION);
  return combine_speed_limit.GetSpeedLimitInfoByS(s);
}

std::optional<SpeedLimitInfo>
SpeedLimitProvider::GetDynamicSpeedLimitInfoByTimeAndS(double t,
                                                       double s) const {
  const int time_idx = static_cast<int>(t / time_step_);
  if (time_idx >= dynamic_speed_limit_.size()) return std::nullopt;
  if (!dynamic_speed_limit_[time_idx].has_value()) return std::nullopt;
  return dynamic_speed_limit_[time_idx]->GetSpeedLimitInfoByS(s);
}

std::optional<SpeedLimitInfo> SpeedLimitProvider::GetVtSpeedLimitInfoByTime(
    double t) const {
  if (min_vt_speed_limit_.empty()) return std::nullopt;
  const int time_idx =
      std::clamp(static_cast<int>(t / time_step_), 0,
                 static_cast<int>(min_vt_speed_limit_.size()) - 1);
  return min_vt_speed_limit_[time_idx];
}

std::optional<SpeedLimitInfo>
SpeedLimitProvider::GetVtSpeedLimitInfoByTypeAndTime(
    const SpeedLimitTypeProto::Type type, double t) const {
  if (vt_speed_limit_map_.find(type) == vt_speed_limit_map_.end()) {
    return std::nullopt;
  }
  const auto& speed_limit_info = FindOrDie(vt_speed_limit_map_, type);
  if (speed_limit_info.empty()) return std::nullopt;
  const int time_idx =
      std::clamp(static_cast<int>(t / time_step_), 0,
                 static_cast<int>(speed_limit_info.size()) - 1);
  return speed_limit_info[time_idx];
}

}  // namespace qcraft::planner
