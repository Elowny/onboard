find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_FREESPACE_PROTO_SRCS ONBOARD_PLANNER_FREESPACE_PROTO_HDRS
  freespace_planner.proto
)
protobuf_generate_python(ONBOARD_PLANNER_FREESPACE_PROTO_PY
  freespace_planner.proto
)

add_custom_target(onboard_planner_freespace_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_FREESPACE_PROTO_PY})
set_target_properties(onboard_planner_freespace_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/FreeSpace/Proto")

add_library(onboard_planner_freespace_proto STATIC
  ${ONBOARD_PLANNER_FREESPACE_PROTO_SRCS}
  ${ONBOARD_PLANNER_FREESPACE_PROTO_HDRS}
)
set_target_properties(onboard_planner_freespace_proto PROPERTIES FOLDER "OnBoard/Planner/FreeSpace/Proto")
target_include_directories(onboard_planner_freespace_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_freespace_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_freespace_proto PUBLIC
  onboard_proto_trajectory_point_proto
  onboard_math_geometry_proto
  onboard_planner_optimization_proto
  onboard_planner_speed_proto
  onboard_planner_decision_proto
  ${Protobuf_LIBS}
)