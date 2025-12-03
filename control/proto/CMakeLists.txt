find_package(Protobuf REQUIRED)
protobuf_generate_cpp(CONTROL_PROTO_SRCS CONTROL_PROTO_HDRS
  controller_conf.proto
  controller_msg.proto
)
protobuf_generate_python(CONTROL_PROTO_PY
  controller_conf.proto
  controller_msg.proto
)

add_custom_target(onboard_control_proto_py_code ALL DEPENDS ${CONTROL_PROTO_PY})
set_target_properties(onboard_control_proto_py_code PROPERTIES FOLDER "OnBoard/Control/Proto")

add_library(onboard_control_proto STATIC
  ${CONTROL_PROTO_SRCS}
  ${CONTROL_PROTO_HDRS}
)
set_target_properties(onboard_control_proto PROPERTIES FOLDER "OnBoard/Control/Proto")
target_include_directories(onboard_control_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_control_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_control_proto PUBLIC
  onboard_math_proto
  onboard_proto_pad_msg_proto
  ${Protobuf_LIBS}
)