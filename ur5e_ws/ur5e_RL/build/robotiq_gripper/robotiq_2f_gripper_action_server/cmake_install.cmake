# Install script for directory: /home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_2f_gripper_action_server

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ben/work/ur5e_RL/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ben/work/ur5e_RL/build/robotiq_gripper/robotiq_2f_gripper_action_server/catkin_generated/installspace/robotiq_2f_gripper_action_server.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_action_server/cmake" TYPE FILE FILES
    "/home/ben/work/ur5e_RL/build/robotiq_gripper/robotiq_2f_gripper_action_server/catkin_generated/installspace/robotiq_2f_gripper_action_serverConfig.cmake"
    "/home/ben/work/ur5e_RL/build/robotiq_gripper/robotiq_2f_gripper_action_server/catkin_generated/installspace/robotiq_2f_gripper_action_serverConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_action_server" TYPE FILE FILES "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_2f_gripper_action_server/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server" TYPE EXECUTABLE FILES "/home/ben/work/ur5e_RL/devel/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_node"
         OLD_RPATH "/opt/ros/noetic/lib:/home/ben/work/ur5e_RL/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_client_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_client_test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_client_test"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server" TYPE EXECUTABLE FILES "/home/ben/work/ur5e_RL/devel/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_client_test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_client_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_client_test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_client_test"
         OLD_RPATH "/opt/ros/noetic/lib:/home/ben/work/ur5e_RL/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server_client_test")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/robotiq_2f_gripper_action_server" TYPE DIRECTORY FILES "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_2f_gripper_action_server/include/robotiq_2f_gripper_action_server/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

