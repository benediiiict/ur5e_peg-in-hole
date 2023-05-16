# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robotiq_3f_gripper_articulated_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Irobotiq_3f_gripper_articulated_msgs:/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotiq_3f_gripper_articulated_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg" NAME_WE)
add_custom_target(_robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotiq_3f_gripper_articulated_msgs" "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg" ""
)

get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg" NAME_WE)
add_custom_target(_robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotiq_3f_gripper_articulated_msgs" "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robotiq_3f_gripper_articulated_msgs
  "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
)
_generate_msg_cpp(robotiq_3f_gripper_articulated_msgs
  "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(robotiq_3f_gripper_articulated_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotiq_3f_gripper_articulated_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages robotiq_3f_gripper_articulated_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg" NAME_WE)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_cpp _robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg" NAME_WE)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_cpp _robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_3f_gripper_articulated_msgs_gencpp)
add_dependencies(robotiq_3f_gripper_articulated_msgs_gencpp robotiq_3f_gripper_articulated_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_3f_gripper_articulated_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robotiq_3f_gripper_articulated_msgs
  "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
)
_generate_msg_eus(robotiq_3f_gripper_articulated_msgs
  "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(robotiq_3f_gripper_articulated_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robotiq_3f_gripper_articulated_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages robotiq_3f_gripper_articulated_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg" NAME_WE)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_eus _robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg" NAME_WE)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_eus _robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_3f_gripper_articulated_msgs_geneus)
add_dependencies(robotiq_3f_gripper_articulated_msgs_geneus robotiq_3f_gripper_articulated_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_3f_gripper_articulated_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robotiq_3f_gripper_articulated_msgs
  "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
)
_generate_msg_lisp(robotiq_3f_gripper_articulated_msgs
  "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(robotiq_3f_gripper_articulated_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robotiq_3f_gripper_articulated_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages robotiq_3f_gripper_articulated_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg" NAME_WE)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_lisp _robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg" NAME_WE)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_lisp _robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_3f_gripper_articulated_msgs_genlisp)
add_dependencies(robotiq_3f_gripper_articulated_msgs_genlisp robotiq_3f_gripper_articulated_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_3f_gripper_articulated_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robotiq_3f_gripper_articulated_msgs
  "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
)
_generate_msg_nodejs(robotiq_3f_gripper_articulated_msgs
  "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(robotiq_3f_gripper_articulated_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robotiq_3f_gripper_articulated_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages robotiq_3f_gripper_articulated_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg" NAME_WE)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_nodejs _robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg" NAME_WE)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_nodejs _robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_3f_gripper_articulated_msgs_gennodejs)
add_dependencies(robotiq_3f_gripper_articulated_msgs_gennodejs robotiq_3f_gripper_articulated_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_3f_gripper_articulated_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robotiq_3f_gripper_articulated_msgs
  "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
)
_generate_msg_py(robotiq_3f_gripper_articulated_msgs
  "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(robotiq_3f_gripper_articulated_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotiq_3f_gripper_articulated_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages robotiq_3f_gripper_articulated_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg" NAME_WE)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_py _robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg" NAME_WE)
add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_py _robotiq_3f_gripper_articulated_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_3f_gripper_articulated_msgs_genpy)
add_dependencies(robotiq_3f_gripper_articulated_msgs_genpy robotiq_3f_gripper_articulated_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_3f_gripper_articulated_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_3f_gripper_articulated_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robotiq_3f_gripper_articulated_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
