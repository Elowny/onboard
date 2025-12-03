#include "onboard/planner/test_util/planner_test_setup.h"

#include <ostream>
#include <string>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/map_selector.h"

namespace qcraft {
namespace planner {

void PlannerTestSetup::set_map(std::string_view map_name) {
  QCHECK_EQ(map_name, "dojo") << "Only dojo map is supported now";
  SetMap(std::string(map_name));
  map_.LoadWholeMap().Build();
}

}  // namespace planner
}  // namespace qcraft
