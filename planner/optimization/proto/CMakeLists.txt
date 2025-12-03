find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_OPTIMIZATION_PROTO_SRCS ONBOARD_PLANNER_OPTIMIZATION_PROTO_HDRS
  optimizer.proto
)
protobuf_generate_python(ONBOARD_PLANNER_OPTIMIZATION_PROTO_PY
  optimizer.proto
)

add_custom_target(onboard_planner_optimization_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_OPTIMIZATION_PROTO_PY})
set_target_properties(onboard_planner_optimization_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Optimization/Proto")

add_library(onboard_planner_optimization_proto STATIC
  ${ONBOARD_PLANNER_OPTIMIZATION_PROTO_SRCS}
  ${ONBOARD_PLANNER_OPTIMIZATION_PROTO_HDRS}
)
set_target_properties(onboard_planner_optimization_proto PROPERTIES FOLDER "OnBoard/Planner/Optimization/Proto")
target_include_directories(onboard_planner_optimization_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_optimization_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_optimization_proto PUBLIC
  onboard_proto_trajectory_point_proto
  onboard_math_geometry_proto
  ${Protobuf_LIBS}
)