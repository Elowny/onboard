find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_INITIALIZER_PROTO_SRCS ONBOARD_PLANNER_INITIALIZER_PROTO_HDRS
  initializer.proto
  initializer_config.proto
)
protobuf_generate_python(ONBOARD_PLANNER_INITIALIZER_PROTO_PY
  initializer.proto
  initializer_config.proto
)
add_custom_target(onboard_planner_initializer_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_INITIALIZER_PROTO_PY})
set_target_properties(onboard_planner_initializer_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Initializer/Proto")

add_library(onboard_planner_initializer_proto STATIC
  ${ONBOARD_PLANNER_INITIALIZER_PROTO_SRCS}
  ${ONBOARD_PLANNER_INITIALIZER_PROTO_HDRS}
)
set_target_properties(onboard_planner_initializer_proto PROPERTIES FOLDER "OnBoard/Planner/Initializer/Proto")
target_include_directories(onboard_planner_initializer_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_initializer_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_initializer_proto PUBLIC
  onboard_planner_decision_proto
  onboard_math_geometry_proto
  onboard_proto_trajectory_point_proto
  ${Protobuf_LIBS}
)

