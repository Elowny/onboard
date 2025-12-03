find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_PLAN_PROTO_SRCS ONBOARD_PLANNER_PLAN_PROTO_HDRS
  plan_task.proto
)
protobuf_generate_python(ONBOARD_PLANNER_PLAN_PROTO_PY
  plan_task.proto
)
add_custom_target(onboard_planner_plan_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_PLAN_PROTO_PY})
set_target_properties(onboard_planner_plan_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Plan/Proto")

add_library(onboard_planner_plan_proto STATIC
  ${ONBOARD_PLANNER_PLAN_PROTO_SRCS}
  ${ONBOARD_PLANNER_PLAN_PROTO_HDRS}
)
set_target_properties(onboard_planner_plan_proto PROPERTIES FOLDER "OnBoard/Planner/Plan/Proto")
target_include_directories(onboard_planner_plan_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_plan_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_plan_proto PUBLIC
  onboard_maps_proto
  common_proto
  ${Protobuf_LIBS}
)