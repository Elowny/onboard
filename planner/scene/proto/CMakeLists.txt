find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_SCENE_PROTO_SRCS ONBOARD_PLANNER_SCENE_PROTO_HDRS
  scene_understanding.proto
)
protobuf_generate_python(ONBOARD_PLANNER_SCENE_PROTO_PY
  scene_understanding.proto
)
add_custom_target(onboard_planner_scene_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_SCENE_PROTO_PY})
set_target_properties(onboard_planner_scene_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Scene/Proto")

add_library(onboard_planner_scene_proto STATIC
  ${ONBOARD_PLANNER_SCENE_PROTO_SRCS}
  ${ONBOARD_PLANNER_SCENE_PROTO_HDRS}
)
set_target_properties(onboard_planner_scene_proto PROPERTIES FOLDER "OnBoard/Planner/Scene/Proto")
target_include_directories(onboard_planner_scene_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_scene_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_scene_proto PUBLIC
  onboard_proto_perception_proto
  onboard_proto_prediction_proto
  onboard_math_geometry_proto
  onboard_maps_proto
  common_proto
  ${Protobuf_LIBS}
)