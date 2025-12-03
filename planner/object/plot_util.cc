#include "onboard/planner/object/plot_util.h"

#include <cmath>
#include <vector>

#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/vis/canvas/canvas.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft::planner {
constexpr double kEps = 1e-6;

void DrawBox2dToCanvas(vis::Canvas* canvas, const Box2d& box,
                       const vis::Color& color) {
  canvas->DrawBox(Vec3d(box.center()), box.heading(),
                  Vec2d(box.length(), box.width()), color, 3);
  const Vec2d front_center = box.center() + box.half_length() * box.tangent();
  canvas->DrawLine(Vec3d(front_center),
                   Vec3d(front_center + 0.15 * box.length() * box.tangent()),
                   color, 3);
}

void DrawPlannerObjectManagerToCanvas(const PlannerObjectManager& object_mgr,
                                      const std::string& channel,
                                      const vis::Color& color) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);

  for (const auto& object : object_mgr.planner_objects()) {
    const auto& bbox = object.bounding_box();
    DrawBox2dToCanvas(&canvas, bbox, color);
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

void DrawPredictionToCanvas(const ObjectsPredictionProto& prediction,
                            const std::string& channel,
                            const vis::Color& color) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);

  for (const auto& object_pred : prediction.objects()) {
    const auto& bbox_proto = object_pred.perception_object().bounding_box();
    DrawBox2dToCanvas(&canvas, Box2d(bbox_proto), color);
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

void DrawSpacetimeObjectTrajectory(const SpacetimeObjectTrajectory& st_obj_traj,
                                   const std::string& channel,
                                   const vis::Color& color) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto& state : st_obj_traj.states()) {
    canvas.DrawPolygon(state.contour, state.traj_point->t() * 10.0, color);
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

void DrawStTrajWithColorableAccel(const SpacetimeObjectTrajectory& st_obj_traj,
                                  const std::string& channel) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto& state : st_obj_traj.states()) {
    if (std::abs(state.traj_point->a()) < kEps) {
      canvas.DrawPolygon(state.contour, state.traj_point->t() * 10.0,
                         vis::Color::kAzure);
    } else if (state.traj_point->a() > kEps) {
      canvas.DrawPolygon(state.contour, state.traj_point->t() * 10.0,
                         vis::Color::kDarkGreen);
    } else {
      canvas.DrawPolygon(state.contour, state.traj_point->t() * 10.0,
                         vis::Color::kCrimson);
    }
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

void ClearCanvasServerBuffers() {
  vis::vantage::GetCanvasClient()->ClearServerBuffers();
}

}  // namespace qcraft::planner
