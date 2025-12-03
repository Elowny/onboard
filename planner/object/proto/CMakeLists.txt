find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_OBJECT_PROTO_SRCS ONBOARD_PLANNER_OBJECT_PROTO_HDRS
  planner_object.proto
)
protobuf_generate_python(ONBOARD_PLANNER_OBJECT_PROTO_PY
  planner_object.proto
)

add_custom_target(onboard_planner_object_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_OBJECT_PROTO_PY})
set_target_properties(onboard_planner_object_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Object/Proto")

add_library(onboard_planner_object_proto STATIC
  ${ONBOARD_PLANNER_OBJECT_PROTO_SRCS}
  ${ONBOARD_PLANNER_OBJECT_PROTO_HDRS}
)
set_target_properties(onboard_planner_object_proto PROPERTIES FOLDER "OnBoard/Planner/Object/Proto")
target_include_directories(onboard_planner_object_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_object_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_object_proto PUBLIC
  onboard_proto_trajectory_point_proto
  onboard_proto_perception_proto
  ${Protobuf_LIBS}
)