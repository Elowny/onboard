#ifndef ONBOARD_PLANNER_OBJECT_PLOT_UTIL_H_
#define ONBOARD_PLANNER_OBJECT_PLOT_UTIL_H_

#include <string>

#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {

void DrawPlannerObjectManagerToCanvas(const PlannerObjectManager& object_mgr,
                                      const std::string& channel,
                                      const vis::Color& color);

void DrawPredictionToCanvas(const ObjectsPredictionProto& prediction,
                            const std::string& channel,
                            const vis::Color& color);

void DrawSpacetimeObjectTrajectory(const SpacetimeObjectTrajectory& st_obj_traj,
                                   const std::string& channel,
                                   const vis::Color& color);

void DrawStTrajWithColorableAccel(const SpacetimeObjectTrajectory& st_obj_traj,
                                  const std::string& channel);

void ClearCanvasServerBuffers();

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_OBJECT_PLOT_UTIL_H_
