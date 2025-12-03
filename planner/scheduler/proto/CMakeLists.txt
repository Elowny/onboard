find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_SCHEDULER_PROTO_SRCS ONBOARD_PLANNER_SCHEDULER_PROTO_HDRS
  lane_change.proto
  reference_line_qp_smoother.proto
  scheduler.proto
  smooth_reference_line.proto
)
protobuf_generate_python(ONBOARD_PLANNER_SCHEDULER_PROTO_PY
  lane_change.proto
  reference_line_qp_smoother.proto
  scheduler.proto
  smooth_reference_line.proto
)

add_custom_target(onboard_planner_scheduler_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_SCHEDULER_PROTO_PY})
set_target_properties(onboard_planner_scheduler_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Scheduler/Proto")

add_library(onboard_planner_scheduler_proto STATIC
  ${ONBOARD_PLANNER_SCHEDULER_PROTO_SRCS}
  ${ONBOARD_PLANNER_SCHEDULER_PROTO_HDRS}
)
set_target_properties(onboard_planner_scheduler_proto PROPERTIES FOLDER "OnBoard/Planner/Scheduler/Proto")
target_include_directories(onboard_planner_scheduler_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_scheduler_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_scheduler_proto PUBLIC
  common_proto
  onboard_maps_proto
  onboard_math_geometry_proto
  ${Protobuf_LIBS}
)