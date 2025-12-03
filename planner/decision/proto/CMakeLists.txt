find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_DECISION_PROTO_SRCS ONBOARD_PLANNER_DECISION_PROTO_HDRS
  constraint.proto
  crosswalk_state.proto
  stop_sign_state.proto
  traffic_light_info.proto
)
protobuf_generate_python(ONBOARD_PLANNER_DECISION_PROTO_PY
  constraint.proto
  crosswalk_state.proto
  stop_sign_state.proto
  traffic_light_info.proto
)
add_custom_target(onboard_planner_decision_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_DECISION_PROTO_PY})
set_target_properties(onboard_planner_decision_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Decision/Proto")

add_library(onboard_planner_decision_proto STATIC
  ${ONBOARD_PLANNER_DECISION_PROTO_SRCS}
  ${ONBOARD_PLANNER_DECISION_PROTO_HDRS}
)
set_target_properties(onboard_planner_decision_proto PROPERTIES FOLDER "OnBoard/Planner/Decision/Proto")
target_include_directories(onboard_planner_decision_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_decision_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_decision_proto PUBLIC
  onboard_proto_perception_proto
  onboard_math_geometry_proto
  onboard_maps_proto
  onboard_math_proto
  common_proto
  ${Protobuf_LIBS}
)