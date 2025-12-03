find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_ML_OPTIMIZER_AUTOTUNING_PROTO_SRCS ONBOARD_PLANNER_ML_OPTIMIZER_AUTOTUNING_PROTO_HDRS
  auto_tuning.proto
)
protobuf_generate_python(ONBOARD_PLANNER_ML_OPTIMIZER_AUTOTUNING_PROTO_PY
  auto_tuning.proto
)
add_custom_target(onboard_planner_ml_optimizer_auto_tuning_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_ML_OPTIMIZER_AUTOTUNING_PROTO_PY})
set_target_properties(onboard_planner_ml_optimizer_auto_tuning_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Ml/OptimizerAutoTuning/Proto")

add_library(onboard_planner_ml_optimizer_auto_tuning_proto STATIC
  ${ONBOARD_PLANNER_ML_OPTIMIZER_AUTOTUNING_PROTO_SRCS}
  ${ONBOARD_PLANNER_ML_OPTIMIZER_AUTOTUNING_PROTO_HDRS}
)
set_target_properties(onboard_planner_ml_optimizer_auto_tuning_proto PROPERTIES FOLDER "OnBoard/Planner/Ml/OptimizerAutoTuning/Proto")
target_include_directories(onboard_planner_ml_optimizer_auto_tuning_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_ml_optimizer_auto_tuning_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_ml_optimizer_auto_tuning_proto PUBLIC
  onboard_proto_trajectory_proto
  onboard_planner_ml_proto
  ${Protobuf_LIBS}
)