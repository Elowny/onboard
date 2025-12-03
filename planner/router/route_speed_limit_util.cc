#include "onboard/planner/router/route_speed_limit_util.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <iterator>
#include <memory>
#include <ostream>

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/util.h"
#include "onboard/planner/router/router_defs.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner {
namespace {
enum class SpeedLimitMode {
  kFractionSpeed = 0,
  kMinSpeed = 1,
  kMaxSpeed = 2,
  kAverageSpeed = 3,
};

bool NeedOverwriteLaneSpeed(mapping::LaneProto::Type type) {
  switch (type) {
    case mapping::LaneProto::GENERAL:
    case mapping::LaneProto::BUS_ONLY:
    case mapping::LaneProto::VIRTUAL:
    case mapping::LaneProto::RAMP:
    case mapping::LaneProto::ENTRANCE:
    case mapping::LaneProto::EXIT:
    case mapping::LaneProto::RAMP_ENTRANCE:
    case mapping::LaneProto::RAMP_EXIT:
    case mapping::LaneProto::RAMP_JCT:
    case mapping::LaneProto::TIDAL:
    case mapping::LaneProto::VARIABLE_DIRECTION:
      return true;
    case mapping::LaneProto::BICYCLE_ONLY:
    case mapping::LaneProto::EMERGENCY:
    case mapping::LaneProto::WALKING_STREET:
    case mapping::LaneProto::PARKING:
    case mapping::LaneProto::MIXED_WITH_CYCLIST:
      return false;
  }
}

double GetOverwrittenLaneSpeedLimitInternal(
    const mapping::v2::SemanticMapManager& smm, mapping::ElementId lane_id,
    double fraction, SpeedLimitMode mode) {
  SMM_LANE_PROTO_OR_RETURN(lane_proto_sp, smm, lane_id, kDefaultSpeedLimit);

  double map_speed_limit = 0.0;
  switch (mode) {
    case SpeedLimitMode::kFractionSpeed: {
      map_speed_limit = GetLaneFractionSpeedLimit(*lane_proto_sp, fraction);
      break;
    }
    case SpeedLimitMode::kMinSpeed: {
      map_speed_limit = GetLaneMinSpeedLimit(*lane_proto_sp);
      break;
    }
    case SpeedLimitMode::kMaxSpeed: {
      map_speed_limit = GetLaneMaxSpeedLimit(*lane_proto_sp);
      break;
    }
    case SpeedLimitMode::kAverageSpeed: {
      map_speed_limit = GetLaneAverageSpeedLimit(*lane_proto_sp);
      break;
    }
  }
  if (smm.GetDataSource() != OnlineMapProto::NAVINFO_HDMAP ||
      !NeedOverwriteLaneSpeed(lane_proto_sp->type())) {
    return map_speed_limit;
  }

  double overwrite_speed_limit = map_speed_limit;
  const auto section =
      smm.FindSection(mapping::SectionId(lane_proto_sp->section_id()));

  if (section != nullptr) {
    switch (section->proto().road_class()) {
      case mapping::SectionProto::NORMAL:
      case mapping::SectionProto::CITY_EXPRESS:
        break;
      case mapping::SectionProto::HIGHWAY: {
        if (map_speed_limit < kHighwayLowestSpeedLimit) {
          overwrite_speed_limit = kHighwayOverwrittenSpeedLimit;
        }
        break;
      }
    }
  }

  switch (lane_proto_sp->type()) {
    case mapping::LaneProto::RAMP:
      return std::max(map_speed_limit, FLAGS_route_overwrite_ramp_speed_kph);
    case mapping::LaneProto::ENTRANCE:
    case mapping::LaneProto::EXIT:
    case mapping::LaneProto::RAMP_ENTRANCE:
    case mapping::LaneProto::RAMP_EXIT:
    case mapping::LaneProto::RAMP_JCT:
      return std::max(map_speed_limit,
                      FLAGS_route_overwrite_highway_junction_speed_kph);

    default:
      break;
  }

  return overwrite_speed_limit;
}

}  // namespace

double GetLaneFractionSpeedLimit(const mapping::LaneProto& lane_proto,
                                 double fraction) {
  const auto& speed_limits_kph = lane_proto.speed_limits_kph();
  if (speed_limits_kph.empty()) {
    return lane_proto.has_speed_limit_kph() ? lane_proto.speed_limit_kph()
                                            : kDefaultSpeedLimit;
  }
  auto iter = std::lower_bound(
      speed_limits_kph.begin(), speed_limits_kph.end(), fraction,
      [](const auto& speed_limit, const double fraction) {
        return speed_limit.start_fraction() < fraction;
      });

  if (iter == speed_limits_kph.begin()) {
    return iter->speed_limit_kph();
  } else {
    return (--iter)->speed_limit_kph();
  }
}

double GetLaneMinSpeedLimit(const mapping::LaneProto& lane_proto) {
  const auto& speed_limits_kph = lane_proto.speed_limits_kph();
  if (speed_limits_kph.empty()) {
    return lane_proto.has_speed_limit_kph() ? lane_proto.speed_limit_kph()
                                            : kDefaultSpeedLimit;
  }
  const auto iter =
      std::min_element(speed_limits_kph.begin(), speed_limits_kph.end(),
                       [](const auto& s1, const auto& s2) {
                         return s1.speed_limit_kph() < s2.speed_limit_kph();
                       });
  return iter->speed_limit_kph();
}

double GetLaneMaxSpeedLimit(const mapping::LaneProto& lane_proto) {
  const auto& speed_limits_kph = lane_proto.speed_limits_kph();
  if (speed_limits_kph.empty()) {
    return lane_proto.has_speed_limit_kph() ? lane_proto.speed_limit_kph()
                                            : kDefaultSpeedLimit;
  }
  const auto iter =
      std::max_element(speed_limits_kph.begin(), speed_limits_kph.end(),
                       [](const auto& s1, const auto& s2) {
                         return s1.speed_limit_kph() < s2.speed_limit_kph();
                       });
  return iter->speed_limit_kph();
}

double GetLaneAverageSpeedLimit(const mapping::LaneProto& lane_proto) {
  const auto& speed_limits_kph = lane_proto.speed_limits_kph();
  if (speed_limits_kph.empty()) {
    return lane_proto.has_speed_limit_kph() ? lane_proto.speed_limit_kph()
                                            : kDefaultSpeedLimit;
  }
  double time_used = 0.0;
  for (int i = 1; i < speed_limits_kph.size(); ++i) {
    const double delta_length = speed_limits_kph[i].start_fraction() -
                                speed_limits_kph[i - 1].start_fraction();

    if (UNLIKELY(speed_limits_kph[i - 1].speed_limit_kph() == 0.0)) {
      return kDefaultSpeedLimit;
    }
    time_used += (delta_length / speed_limits_kph[i - 1].speed_limit_kph());
  }
  // Process the last speed limit
  if (UNLIKELY(speed_limits_kph.rbegin()->speed_limit_kph() == 0.0)) {
    return kDefaultSpeedLimit;
  }
  time_used += (1.0 - speed_limits_kph.rbegin()->start_fraction()) /
               speed_limits_kph.rbegin()->speed_limit_kph();
  if (UNLIKELY(time_used == 0.0)) {
    return kDefaultSpeedLimit;
  }
  return 1.0 / time_used;
}

double GetOverwrittenLaneFractionSpeedLimit(
    const mapping::v2::SemanticMapManager& smm, mapping::ElementId lane_id,
    double fraction) {
  return GetOverwrittenLaneSpeedLimitInternal(smm, lane_id, fraction,
                                              SpeedLimitMode::kFractionSpeed);
}

double GetOverwrittenLaneMinSpeedLimit(
    const mapping::v2::SemanticMapManager& smm, mapping::ElementId lane_id) {
  return GetOverwrittenLaneSpeedLimitInternal(smm, lane_id, /*fraction=*/0.0,
                                              SpeedLimitMode::kMinSpeed);
}

double GetOverwrittenLaneMaxSpeedLimit(
    const mapping::v2::SemanticMapManager& smm, mapping::ElementId lane_id) {
  return GetOverwrittenLaneSpeedLimitInternal(smm, lane_id, /*fraction=*/0.0,
                                              SpeedLimitMode::kMaxSpeed);
}

double GetOverwrittenLaneAverageSpeedLimit(
    const mapping::v2::SemanticMapManager& smm, mapping::ElementId lane_id) {
  return GetOverwrittenLaneSpeedLimitInternal(smm, lane_id, /*fraction=*/0.0,
                                              SpeedLimitMode::kAverageSpeed);
}

std::optional<double> FindFrontOverwrittenSpeedLimit(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& route_sections, double look_ahead_dist) {
  double residual_look_ahead_dist = look_ahead_dist;

  const auto& sec_ids = route_sections.section_id();
  const mapping::SectionProto* preview_sec = nullptr;
  int i = 0;
  double preview_frac = 0.0;
  for (; i < sec_ids.size(); ++i) {
    SMM_SECTION_PROTO_OR_BREAK(cur_sec_ptr, smm,
                               mapping::SectionId(sec_ids[i]));
    const double start_frac = i == 0 ? route_sections.start_fraction() : 0.0;
    const double end_frac =
        i + 1 == sec_ids.size() ? route_sections.end_fraction() : 1.0;

    const double cur_length =
        cur_sec_ptr->average_length() * (end_frac - start_frac);
    residual_look_ahead_dist -= std::max(0.0, cur_length);

    preview_sec = cur_sec_ptr;
    if (residual_look_ahead_dist <= 0) {
      preview_frac =
          std::clamp(start_frac + (residual_look_ahead_dist + cur_length) /
                                      cur_sec_ptr->average_length(),
                     start_frac, end_frac);
      break;
    }
  }

  if (preview_sec == nullptr) {
    QLOG(ERROR) << "Can not find a valid section info.";
    return std::nullopt;
  }

  double max_speed_limit = 0.0;

  const int next_idx = std::min(sec_ids.size() - 1, i + 1);

  const auto next_sec_ptr =
      smm.FindSection(mapping::SectionId(sec_ids[next_idx]));

  if (next_sec_ptr == nullptr ||
      next_sec_ptr->id().value() == preview_sec->id()) {
    for (const auto& lane_id : preview_sec->lanes()) {
      SMM_LANE_PROTO_OR_CONTINUE(cur_lane_ptr, smm,
                                 mapping::ElementId(lane_id));
      max_speed_limit =
          std::max(max_speed_limit,
                   Kph2Mps(GetOverwrittenLaneFractionSpeedLimit(
                       smm, mapping::ElementId(lane_id), preview_frac)));
    }
  } else {
    const auto& next_sec_proto = next_sec_ptr->proto();
    for (const auto& lane_id : preview_sec->lanes()) {
      SMM_LANE_PROTO_OR_CONTINUE(cur_lane_ptr, smm,
                                 mapping::ElementId(lane_id));
      for (const auto& out_lane_id : cur_lane_ptr->outgoing_lanes()) {
        if (std::find(next_sec_proto.lanes().begin(),
                      next_sec_proto.lanes().end(),
                      out_lane_id) != next_sec_proto.lanes().end()) {
          max_speed_limit = std::max(
              max_speed_limit, Kph2Mps(GetOverwrittenLaneFractionSpeedLimit(
                                   smm, mapping::ElementId(lane_id),
                                   /*fraction=*/0.0)));
        }
      }
    }
  }

  return max_speed_limit;
}
}  // namespace qcraft::planner
