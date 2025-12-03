#include "onboard/planner/object/planner_object_manager_builder.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"

#include "onboard/async/parallel_for.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {

absl::StatusOr<PlannerObjectManager> PlannerObjectManagerBuilder::Build(
    FilteredTrajectories* filtered_trajs, ThreadPool* /*thread_pool*/) {
  SCOPED_QTRACE("PlannerObjectManagerBuilder::Build");

  // Filter trajectories.
  for (auto& object : planner_objects_) {
    auto& trajs = *object.mutable_prediction()->mutable_trajectories();
    int i = 0;
    for (int j = 0, n = trajs.size(); j < n; ++j) {
      auto& traj = trajs[j];
      bool is_traj_filtered = false;
      for (const auto* filter : filters_) {
        const auto reason = filter->Filter(object, traj);
        if (reason != FilterReason::NONE) {
          if (filtered_trajs != nullptr) {
            auto* filtered = filtered_trajs->add_filtered();
            filtered->set_reason(reason);
            filtered->set_id(object.id());
            filtered->set_index(traj.index());
          }
          is_traj_filtered = true;
          break;
        }
      }
      if (!is_traj_filtered) {
        // This is equivalent to erasing the filtered element, but can reduce
        // the number of element moves.
        if (i != j) {
          trajs[i] = std::move(traj);
        }
        ++i;
      }
    }
    trajs.erase(trajs.begin() + i, trajs.end());
  }

  planner_objects_.erase(
      std::remove_if(planner_objects_.begin(), planner_objects_.end(),
                     [](const auto& obj) { return obj.num_trajs() == 0; }),
      planner_objects_.end());

  return PlannerObjectManager(std::move(planner_objects_));
}

ObjectVector<PlannerObject> BuildPlannerObjectsFromObjectPrediction(
    const ObjectsProto* perception,
    const prediction::ObjectsPrediction* prediction,
    std::optional<double> align_time, ThreadPool* thread_pool) {
  SCOPED_QTRACE_ARG2("BuildPlannerObjects", "num_objects",
                     perception == nullptr ? 0 : perception->objects_size(),
                     "num_predictions",
                     prediction == nullptr ? 0 : prediction->size());

  struct ObjectInfo {
    std::string_view id;
    const ObjectProto* obj_ptr = nullptr;
    const prediction::ObjectPrediction* pred_ptr = nullptr;
  };

  absl::flat_hash_map<std::string_view, int> id_to_index;
  std::vector<ObjectInfo> obj_info;
  obj_info.reserve((perception == nullptr ? 0 : perception->objects_size()) +
                   (prediction == nullptr ? 0 : prediction->size()));

  if (perception != nullptr) {
    for (const auto& obj : perception->objects()) {
      id_to_index[obj.id()] = obj_info.size();
      obj_info.push_back(
          {.id = obj.id(), .obj_ptr = &obj, .pred_ptr = nullptr});
    }
  }

  if (prediction != nullptr) {
    for (const auto& obj_pred : *prediction) {
      const auto& id = obj_pred.perception_object().id();
      if (const auto* index = FindOrNull(id_to_index, id); index != nullptr) {
        obj_info[*index].pred_ptr = &obj_pred;
      } else {
        obj_info.push_back(
            {.id = id, .obj_ptr = nullptr, .pred_ptr = &obj_pred});
      }
    }
  }

  ObjectVector<PlannerObject> planner_objects;
  planner_objects.resize(obj_info.size());

  ParallelFor(0, obj_info.size(), thread_pool, [&](int i) {
    // When there is no prediction, create one.
    if (obj_info[i].pred_ptr == nullptr) {
      if (auto object_pred_or = prediction::InstantObjectPredictionForNewObject(
              *obj_info[i].obj_ptr,
              /*prediction_time=*/3.0);
          object_pred_or.ok()) {
        const double time_shift =
            align_time.has_value()
                ? *align_time - obj_info[i].obj_ptr->timestamp()
                : 0.0;
        bool shift_success = object_pred_or->ShiftTimeWithObjectProto(
            /*prediction_shift_time=*/time_shift,
            /*object_shift_time=*/time_shift, *obj_info[i].obj_ptr);
        if (shift_success) {
          planner_objects[ObjectIndex(i)] =
              PlannerObject(std::move(*object_pred_or));
        }
        // If shift is unsuccessful, it means that empty prediction result is
        // created.
      } else {
        QLOG(ERROR) << object_pred_or.status();
      }
      return;
    }

    // In the following code, pred_ptr is not NULL.
    const ObjectProto* latest_obj = nullptr;
    if (obj_info[i].obj_ptr != nullptr &&
        obj_info[i].obj_ptr->timestamp() >
            obj_info[i].pred_ptr->perception_object().timestamp()) {
      // Use the latest perception if available.
      latest_obj = obj_info[i].obj_ptr;
    } else {
      latest_obj = &obj_info[i].pred_ptr->perception_object();
    }
    if (latest_obj != nullptr) {
      const double object_time_shift =
          align_time.has_value() ? *align_time - latest_obj->timestamp() : 0.0;
      const double prediction_time_shift =
          align_time.has_value()
              ? *align_time -
                    obj_info[i].pred_ptr->perception_object().timestamp()
              : 0.0;
      // TODO(changqing): Do not use copy.
      auto object_pred_to_shift = *obj_info[i].pred_ptr;
      if (object_pred_to_shift.ShiftTimeWithObjectProto(
              prediction_time_shift, object_time_shift, *latest_obj)) {
        planner_objects[ObjectIndex(i)] =
            PlannerObject(std::move(object_pred_to_shift));
      }
      // Do not add to planner object container if shifting object prediction
      // fails (which create empty trajectories).
    }
  });

  // It should never trigger since shifting ObjectPrediction checked the
  // shifting status.

  planner_objects.erase(
      std::remove_if(
          planner_objects.begin(), planner_objects.end(),
          [](const PlannerObject& obj) { return obj.num_trajs() == 0; }),
      planner_objects.end());

  return planner_objects;
}

ObjectVector<PlannerObject> BuildPlannerObjects(
    const ObjectsProto* perception, const ObjectsPredictionProto* prediction,
    std::optional<double> align_time, ThreadPool* thread_pool) {
  SCOPED_QTRACE_ARG2("BuildPlannerObjects", "num_objects",
                     perception == nullptr ? 0 : perception->objects_size(),
                     "num_predictions",
                     prediction == nullptr ? 0 : prediction->objects_size());

  struct ObjectInfo {
    std::string_view id;
    const ObjectProto* obj_ptr;
    const ObjectPredictionProto* pred_ptr;
  };
  absl::flat_hash_map<std::string_view, int> id_to_index;
  std::vector<ObjectInfo> obj_info;
  obj_info.reserve((perception == nullptr ? 0 : perception->objects_size()) +
                   (prediction == nullptr ? 0 : prediction->objects_size()));
  if (perception != nullptr) {
    for (const auto& obj : perception->objects()) {
      id_to_index[obj.id()] = obj_info.size();
      obj_info.push_back(
          {.id = obj.id(), .obj_ptr = &obj, .pred_ptr = nullptr});
    }
  }
  if (prediction != nullptr) {
    for (const auto& pred : prediction->objects()) {
      const auto& id = pred.perception_object().id();
      if (const auto* index = FindOrNull(id_to_index, id); index != nullptr) {
        obj_info[*index].pred_ptr = &pred;
      } else {
        obj_info.push_back({.id = id, .obj_ptr = nullptr, .pred_ptr = &pred});
      }
    }
  }
  ObjectVector<PlannerObject> planner_objects;
  planner_objects.resize(obj_info.size());
  ParallelFor(0, obj_info.size(), thread_pool, [&](int i) {
    // When there is no prediction, create one.
    if (obj_info[i].pred_ptr == nullptr) {
      if (auto result = prediction::InstantPredictionForNewObject(
              *obj_info[i].obj_ptr,
              /*prediction_time=*/3.0);
          result.ok()) {
        const double time_shift =
            align_time.has_value()
                ? *align_time - obj_info[i].obj_ptr->timestamp()
                : 0.0;
        planner_objects[ObjectIndex(i)] =
            PlannerObject(prediction::ObjectPrediction(
                *result, /*prediction_shift_time=*/time_shift,
                *obj_info[i].obj_ptr, /*object_shift_time=*/time_shift));
      } else {
        QLOG(ERROR) << result.status();
      }
      return;
    }

    // In the following code, pred_ptr is not NULL.
    const ObjectProto* latest_obj = nullptr;
    if (obj_info[i].obj_ptr != nullptr &&
        obj_info[i].obj_ptr->timestamp() >
            obj_info[i].pred_ptr->perception_object().timestamp()) {
      latest_obj = obj_info[i].obj_ptr;
    } else if (obj_info[i].pred_ptr->has_perception_object()) {
      latest_obj = &obj_info[i].pred_ptr->perception_object();
    }

    if (latest_obj != nullptr) {
      const double object_time_shift =
          align_time.has_value() ? *align_time - latest_obj->timestamp() : 0.0;
      const double prediction_time_shift =
          align_time.has_value()
              ? *align_time -
                    obj_info[i].pred_ptr->perception_object().timestamp()
              : 0.0;
      planner_objects[ObjectIndex(i)] =
          PlannerObject(prediction::ObjectPrediction(
              *obj_info[i].pred_ptr, prediction_time_shift, *latest_obj,
              object_time_shift));
    }
  });

  // Trim objects that has no prediction trajectory. It is unlikely to trigger.
  planner_objects.erase(
      std::remove_if(
          planner_objects.begin(), planner_objects.end(),
          [](const PlannerObject& obj) { return obj.num_trajs() == 0; }),
      planner_objects.end());
  return planner_objects;
}

}  // namespace planner
}  // namespace qcraft
