#ifndef ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_ADAPTER_
#define ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_ADAPTER_

#define HAVE_STDDEF_H
#include "coin/IpIpoptApplication.hpp"
#include "coin/IpSolveStatistics.hpp"
#undef HAVE_STDDEF_H
#include <string>       // NOLINT
#include <string_view>  // NOLINT
#include <utility>      // NOLINT
#include <vector>       // NOLINT

#include "absl/status/statusor.h"

#include "onboard/planner/optimization/ipopt/ipopt_optimizer.h"
#include "onboard/planner/optimization/ipopt/ipopt_util.h"
#include "onboard/planner/trajectory_point.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class IpoptAdapter {
 public:
  IpoptAdapter(const PROB* problem, int horizon, std::string_view owner);
  virtual ~IpoptAdapter() = default;

  void SetInitialPoints(std::vector<TrajectoryPoint> init_points) {
    mynlp_->SetInitialPoints(std::move(init_points));
  }

  using HookType = typename IpoptOptimizer<PROB>::HookType;

  void AddHook(HookType* hook) { mynlp_->AddHook(hook); }

  absl::StatusOr<std::vector<TrajectoryPoint>> Solve(std::string* result_info);

 private:
  std::string owner_;
  Ipopt::SmartPtr<IpoptOptimizer<PROB>> mynlp_;
};

template <typename PROB>
IpoptAdapter<PROB>::IpoptAdapter(const PROB* problem, int horizon,
                                 std::string_view owner)
    : owner_(owner), mynlp_(new IpoptOptimizer(problem, horizon, owner)) {}

template <typename PROB>
absl::StatusOr<std::vector<TrajectoryPoint>> IpoptAdapter<PROB>::Solve(
    std::string* result_info) {
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetStringValue("jacobian_approximation", "exact");
  constexpr double kMaxSolveTime = 5.0;  // s
  app->Options()->SetNumericValue("max_cpu_time", kMaxSolveTime);
  constexpr int kIpoptPrintLevel = 5;
  app->Options()->SetIntegerValue("print_level", kIpoptPrintLevel);

  // Initialize the IpoptApplication and process the options
  Ipopt::ApplicationReturnStatus status;
  status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    *result_info = IpoptReturnStatusToString(status);
    return absl::InternalError(
        absl::StrFormat("Ipopt init failed in %s, errorcode(%s)", owner_,
                        IpoptReturnStatusToString(status)));
  }

  status = app->OptimizeTNLP(mynlp_);
  *result_info = IpoptReturnStatusToString(status);

  // Export result
  if (!(status == Ipopt::Solve_Succeeded ||
        status == Ipopt::Feasible_Point_Found ||
        status == Ipopt::Maximum_Iterations_Exceeded ||
        status == Ipopt::Maximum_CpuTime_Exceeded ||
        status == Ipopt::Search_Direction_Becomes_Too_Small ||
        status == Ipopt::Solved_To_Acceptable_Level ||
        status == Ipopt::Restoration_Failed ||
        status == Ipopt::User_Requested_Stop)) {
    return absl::InternalError(
        absl::StrFormat("Ipopt solve failed in %s, errorcode(%s)", owner_,
                        IpoptReturnStatusToString(status)));
  }

  std::vector<TrajectoryPoint> result = mynlp_->GetFinalTraj();
  return result;
}

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_ADAPTER_
