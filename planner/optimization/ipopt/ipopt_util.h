#ifndef ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_UTIL_H_

#include <string>

#include "coin/IpReturnCodes.hpp"  // IWYU pragma: keep

#define HAVE_STDDEF_H
#undef HAVE_STDDEF_H

namespace qcraft {
namespace planner {

std::string IpoptReturnStatusToString(Ipopt::ApplicationReturnStatus status);

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_UTIL_H_
