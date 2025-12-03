#include <cmath>
#include <memory>
#include <optional>
#include <random>
#include <vector>

#include "absl/status/statusor.h"
#include "benchmark/benchmark.h"

#include "common/proto/semantic_map_modifier.pb.h"

#include "onboard/async/future.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"
#include "onboard/planner/plan/acc/acc_corridor_with_map.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kTimes = 100;

double RandomDouble(double start, double end, std::mt19937* gen) {
  std::uniform_real_distribution<> dis(start, end);
  return dis(*gen);
}

Vec2d RandomPointInUnitCircle(std::mt19937* gen) {
  constexpr int kTrialThreshold = 20;
  double x, y;
  for (int i = 0; i < kTrialThreshold; ++i) {
    x = RandomDouble(-1.0, 1.0, gen);
    y = RandomDouble(-1.0, 1.0, gen);

    if (Sqr(x) + Sqr(y) <= 1.0) {
      return Vec2d{x, y};
    }
  }
  // Probability of triggering this line is:
  // P_out^20, P_out = 1 - 3.14/4.0
  // Which evaluates to 5e-14.
  return Vec2d{0.0, 0.0};
}

std::vector<Vec2d> GenerateRandomXYPointsAroundPoint(int n, const Vec2d& pos,
                                                     double radius) {
  std::mt19937 gen;
  std::vector<Vec2d> pts;
  pts.reserve(n);

  for (int i = 0; i < n; ++i) {
    pts.emplace_back(pos + radius * RandomPointInUnitCircle(&gen));
  }

  return pts;
}

void GenerateRandomPlanStartPointAroundXY(
    int n, const Vec2d& pos,
    std::vector<ApolloTrajectoryPointProto>* plan_start_points,
    std::vector<PoseProto>* poses) {
  const auto xys = GenerateRandomXYPointsAroundPoint(n, pos, 1.0);
  std::mt19937 gen_heading;
  std::vector<double> headings;
  headings.reserve(n);
  for (int i = 0; i < n; ++i) {
    headings.push_back(RandomDouble(0, M_PI_2, &gen_heading));
  }

  std::mt19937 gen_v;
  std::vector<double> vs;
  vs.reserve(n);
  for (int i = 0; i < n; ++i) {
    vs.push_back(RandomDouble(0, Kph2Mps(80.0), &gen_v));
  }

  plan_start_points->reserve(n);
  poses->reserve(n);
  for (int i = 0; i < n; ++i) {
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(xys[i].x());
    pose.mutable_pos_smooth()->set_y(xys[i].y());
    pose.set_yaw(headings[i]);
    pose.mutable_vel_body()->set_x(vs[i]);
    poses->push_back(pose);
    plan_start_points->push_back(ConvertToTrajPointProto(pose));
  }
}

void BM_BuildAccCorridorWithMap(benchmark::State& state) {  // NOLINT
  // Generate contexts.
  auto param_manager = CreateParamManagerFromCarId("Q2506");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();

  SetMap("dojo");
  auto loader = mapping::v2::SemanticMapLoader::MakeShared();
  auto map_fut = loader->PreloadWholeMap();
  auto smm = map_fut.Get();
  auto cc = CoordinateConverter::FromMap("dojo");
  mapping::SemanticMapModifierProto sm_mod_proto;
  PlannerSemanticMapModification modifier =
      CreateSemanticMapModification(*smm, sm_mod_proto);
  const auto& psmm = CreateDojoTestPSMM();

  const auto& vehicle_geom = vehicle_param_api.vehicle_geometry_params();
  const auto& vehicle_drive = vehicle_param_api.vehicle_drive_params();

  std::vector<ApolloTrajectoryPointProto> plan_start_points;
  std::vector<PoseProto> poses;
  GenerateRandomPlanStartPointAroundXY(kTimes, Vec2d(0.0, 0.0),
                                       &plan_start_points, &poses);

  for (auto _ : state) {
    const auto idx = state.iterations() % kTimes;
    const auto dm_or =
        BuildDrivingMapTopo(poses[idx], psmm, plan_start_points[idx].v());
    if (!dm_or.ok()) continue;
    benchmark::DoNotOptimize(BuildAccCorridorFromClosestLanePath(
        poses[idx], plan_start_points[idx], psmm, *dm_or,
        /*steering_percentage=*/std::nullopt, vehicle_geom, vehicle_drive,
        /*corridor_step_s=*/2.0,
        /*total_preview_time=*/kAccTrajectoryTimeHorizon));
  }
}
BENCHMARK(BM_BuildAccCorridorWithMap);
}  // namespace
}  // namespace planner
}  // namespace qcraft

BENCHMARK_MAIN();
