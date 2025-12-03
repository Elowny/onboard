#include "onboard/planner/optimization/ipopt/ipopt_util.h"

namespace qcraft {
namespace planner {

/*
  Return detailed failure information,
  reference resource: Ipopt::ApplicationReturnStatus, https://
  www.coin-or.org/Doxygen/CoinAll/_ip_return_codes__inc_8h-source.html
*/
std::string IpoptReturnStatusToString(Ipopt::ApplicationReturnStatus status) {
  switch (static_cast<int>(status)) {
    case 0:
      return "Solve_Succeeded";
    case 1:
      return "Solved_To_Acceptable_Level";
    case 2:
      return "Infeasible_Problem_Detected";
    case 3:
      return "Search_Direction_Becomes_Too_Small";
    case 4:
      return "Diverging_Iterates";
    case 5:
      return "User_Requested_Stop";
    case 6:
      return "Feasible_Point_Found";
    case -1:
      return "Maximum_Iterations_Exceeded";
    case -2:
      return "Restoration_Failed";
    case -3:
      return "Error_In_Step_Computation";
    case -10:
      return "Not_Enough_Degrees_Of_Freedom";
    case -11:
      return "Invalid_Problem_Definition";
    case -12:
      return "Invalid_Option";
    case -13:
      return "Invalid_Number_Detected";
    case -100:
      return "Unrecoverable_Exception";
    case -101:
      return "NonIpopt_Exception_Thrown";
    case -102:
      return "Insufficient_Memory";
    case -199:
      return "Internal_Error";
    default:
      return "Unknown_failure_code";
  }
}

}  // namespace planner
}  // namespace qcraft
