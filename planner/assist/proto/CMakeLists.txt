find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_ASSIST_PROTO_SRCS ONBOARD_PLANNER_ASSIST_PROTO_HDRS
  assist_plan_state.proto
  external_command_status.proto
)
protobuf_generate_python(ONBOARD_PLANNER_ASSIST_PROTO_PY
  assist_plan_state.proto
  external_command_status.proto
)
add_custom_target(onboard_planner_assist_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_ASSIST_PROTO_PY})
set_target_properties(onboard_planner_assist_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Assist/Proto")

add_library(onboard_planner_assist_proto STATIC
  ${ONBOARD_PLANNER_ASSIST_PROTO_SRCS}
  ${ONBOARD_PLANNER_ASSIST_PROTO_HDRS}
)
set_target_properties(onboard_planner_assist_proto PROPERTIES FOLDER "OnBoard/Planner/Assist/Proto")
target_link_libraries(onboard_planner_assist_proto PUBLIC
  onboard_proto_autonomy_state_proto
  onboard_proto_q_run_event_proto
  onboard_proto_driving_style_proto
  onboard_lite_proto
  onboard_maps_proto
  common_qacc_proto
  common_proto
)