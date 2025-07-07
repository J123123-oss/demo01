# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "serial_comms: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iserial_comms:/home/ubuntu/demo01/src/serial_comms/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(serial_comms_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg" NAME_WE)
add_custom_target(_serial_comms_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "serial_comms" "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg" ""
)

get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg" NAME_WE)
add_custom_target(_serial_comms_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "serial_comms" "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(serial_comms
  "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_comms
)
_generate_msg_cpp(serial_comms
  "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_comms
)

### Generating Services

### Generating Module File
_generate_module_cpp(serial_comms
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_comms
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(serial_comms_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(serial_comms_generate_messages serial_comms_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg" NAME_WE)
add_dependencies(serial_comms_generate_messages_cpp _serial_comms_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg" NAME_WE)
add_dependencies(serial_comms_generate_messages_cpp _serial_comms_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(serial_comms_gencpp)
add_dependencies(serial_comms_gencpp serial_comms_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_comms_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(serial_comms
  "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/serial_comms
)
_generate_msg_eus(serial_comms
  "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/serial_comms
)

### Generating Services

### Generating Module File
_generate_module_eus(serial_comms
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/serial_comms
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(serial_comms_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(serial_comms_generate_messages serial_comms_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg" NAME_WE)
add_dependencies(serial_comms_generate_messages_eus _serial_comms_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg" NAME_WE)
add_dependencies(serial_comms_generate_messages_eus _serial_comms_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(serial_comms_geneus)
add_dependencies(serial_comms_geneus serial_comms_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_comms_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(serial_comms
  "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_comms
)
_generate_msg_lisp(serial_comms
  "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_comms
)

### Generating Services

### Generating Module File
_generate_module_lisp(serial_comms
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_comms
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(serial_comms_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(serial_comms_generate_messages serial_comms_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg" NAME_WE)
add_dependencies(serial_comms_generate_messages_lisp _serial_comms_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg" NAME_WE)
add_dependencies(serial_comms_generate_messages_lisp _serial_comms_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(serial_comms_genlisp)
add_dependencies(serial_comms_genlisp serial_comms_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_comms_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(serial_comms
  "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/serial_comms
)
_generate_msg_nodejs(serial_comms
  "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/serial_comms
)

### Generating Services

### Generating Module File
_generate_module_nodejs(serial_comms
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/serial_comms
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(serial_comms_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(serial_comms_generate_messages serial_comms_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg" NAME_WE)
add_dependencies(serial_comms_generate_messages_nodejs _serial_comms_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg" NAME_WE)
add_dependencies(serial_comms_generate_messages_nodejs _serial_comms_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(serial_comms_gennodejs)
add_dependencies(serial_comms_gennodejs serial_comms_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_comms_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(serial_comms
  "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_comms
)
_generate_msg_py(serial_comms
  "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_comms
)

### Generating Services

### Generating Module File
_generate_module_py(serial_comms
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_comms
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(serial_comms_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(serial_comms_generate_messages serial_comms_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/Distances.msg" NAME_WE)
add_dependencies(serial_comms_generate_messages_py _serial_comms_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/demo01/src/serial_comms/msg/INSPVAE.msg" NAME_WE)
add_dependencies(serial_comms_generate_messages_py _serial_comms_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(serial_comms_genpy)
add_dependencies(serial_comms_genpy serial_comms_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_comms_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_comms)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_comms
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(serial_comms_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/serial_comms)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/serial_comms
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(serial_comms_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_comms)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_comms
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(serial_comms_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/serial_comms)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/serial_comms
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(serial_comms_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_comms)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_comms\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_comms
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(serial_comms_generate_messages_py std_msgs_generate_messages_py)
endif()
