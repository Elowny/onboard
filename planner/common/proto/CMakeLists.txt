find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_COMMON_PROTO_SRCS ONBOARD_PLANNER_COMMON_PROTO_HDRS
  planner_status.proto
)
protobuf_generate_python(ONBOARD_PLANNER_COMMON_PROTO_PY
  planner_status.proto
)

add_custom_target(onboard_planner_common_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_COMMON_PROTO_PY})
set_target_properties(onboard_planner_common_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Common/Proto")

add_library(onboard_planner_common_proto STATIC
  ${ONBOARD_PLANNER_COMMON_PROTO_SRCS}
  ${ONBOARD_PLANNER_COMMON_PROTO_HDRS}
)
set_target_properties(onboard_planner_common_proto PROPERTIES FOLDER "OnBoard/Planner/Common/Proto")
target_include_directories(onboard_planner_common_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_common_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_common_proto PUBLIC
  ${Protobuf_LIBS}
)