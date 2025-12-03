#ifndef ONBOARD_PLANNER_ASSIST_TJA_STATE_H_
#define ONBOARD_PLANNER_ASSIST_TJA_STATE_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/planner/assist/proto/tja_state.pb.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft::planner {

struct BoundaryPoint {
  StationBoundaryType type;
  Vec2d point;
};

struct TjaState {
  void FromProto(const TjaStateProto& proto);
  void ToProto(TjaStateProto* proto) const;
  void Reset();

  std::vector<Vec2d> center_line;
  absl::flat_hash_map<std::string, std::vector<Vec2d>> obs_history_pos;
  std::vector<std::string> target_obs_ids;
  bool planner_use_tja_map = false;
  int exit_counter = 0;  // Need to exit tja when counter is 0.
  std::vector<Vec2d> planner_center_line;
  int update_id = 0;
  std::vector<BoundaryPoint> left_boundary;
  std::vector<BoundaryPoint> right_boundary;
  std::vector<std::string> potential_obs_ids;
  std::vector<mapping::ElementId> exit_lane_ids;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ASSIST_TJA_STATE_H_
