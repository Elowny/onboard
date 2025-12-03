find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_SELECTOR_PROTO_SRCS ONBOARD_PLANNER_SELECTOR_PROTO_HDRS
  selector_debug.proto
  selector_params.proto
  selector_state.proto
)
protobuf_generate_python(ONBOARD_PLANNER_SELECTOR_PROTO_PY
  selector_debug.proto
  selector_params.proto
  selector_state.proto
)
add_custom_target(onboard_planner_selector_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_SELECTOR_PROTO_PY})
set_target_properties(onboard_planner_selector_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Selector/Proto")

add_library(onboard_planner_selector_proto STATIC
  ${ONBOARD_PLANNER_SELECTOR_PROTO_SRCS}
  ${ONBOARD_PLANNER_SELECTOR_PROTO_HDRS}
)
set_target_properties(onboard_planner_selector_proto PROPERTIES FOLDER "OnBoard/Planner/Selector/Proto")
target_include_directories(onboard_planner_selector_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_selector_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_selector_proto PUBLIC
  onboard_proto_trajectory_point_proto
  onboard_planner_scheduler_proto
  onboard_prediction_proto
  onboard_planner_ml_condition_feature_extractor_proto
  onboard_proto_turn_signal_proto
  ${Protobuf_LIBS}
)