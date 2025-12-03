#include "onboard/prediction/container/objects_history.h"

// IWYU pragma: no_include <boost/move/utility_core.hpp>  // for move
#include <algorithm>
#include <vector>

#include "boost/circular_buffer.hpp"

#include "onboard/async/parallel_for.h"
#include "onboard/async/thread_pool.h"
#include "onboard/global/trace.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace prediction {
void ObjectsHistory::Update(const ObjectsProto& objects_proto,
                            const LocalizationTransformProto& loc_transform,
                            ThreadPool* thread_pool) {
  SCOPED_QTRACE("ObjectsHistory::Update");
  double end_t = MicroSecondsToSeconds(objects_proto.header().timestamp());
  CoordinateConverter target(loc_transform);

  const int num_objects = objects_proto.objects_size();
  std::vector<int> updated_objs(num_objects, 0);
  for (int i = 0; i < num_objects; ++i) {
    const auto& obj_proto = objects_proto.objects(i);
    const auto* hist = this->FindOrNull(obj_proto.id());
    if (hist != nullptr) {
      double last_t = hist->GetHistory().back().time;
      // If the time difference between new obj and last observation is less
      // than min_dt_, do not add them.
      if (obj_proto.timestamp() - last_t < min_dt_) {
        continue;
      }
    }
    auto& history = (*this)[obj_proto.id()];
    history.Push(obj_proto.timestamp(), PredictionObject(obj_proto, target));
    updated_objs[i] = 1;
    end_t = std::min(obj_proto.timestamp(), end_t);
  }

  end_t -= max_hist_time_len_;

  ParallelFor(0, num_objects, thread_pool, [&](int i) {
    if (updated_objs[i] == 1) {
      const auto& o = objects_proto.objects(i);
      auto& history = (*this)[o.id()];
      auto& buffer = *history.mutable_buffer();
      history.PopBegin(end_t);
      // We don't have to transform the last object.
      for (int j = 0; j + 1 < buffer.size(); ++j) {
        buffer[j].val.TransformCoordinate(target);
      }
      history.UpdateStopTimeInfo();
    }
  });
  PopBegin(end_t);
}

}  // namespace prediction
}  // namespace qcraft
