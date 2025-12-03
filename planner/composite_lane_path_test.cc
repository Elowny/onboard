#include "onboard/planner/composite_lane_path.h"

#include "gtest/gtest.h"

#include "onboard/maps/v2/semantic_map_loader.h"

namespace qcraft::planner {
namespace {
TEST(CompositeLanePathTest, Build) {
  SetMap("dojo");
  auto loader = mapping::v2::SemanticMapLoader::MakeShared();
  auto map_fut = loader->PreloadWholeMap();
  auto smm = map_fut.Get();
  mapping::LanePoint lpt = {mapping::ElementId(3), 0.5};
  CompositeLanePath clp(smm.get(), lpt);
}
}  // namespace
}  // namespace qcraft::planner
