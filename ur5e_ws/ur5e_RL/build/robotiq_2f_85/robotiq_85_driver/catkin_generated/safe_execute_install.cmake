execute_process(COMMAND "/home/ben/work/ur5e_RL/build/robotiq_2f_85/robotiq_85_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ben/work/ur5e_RL/build/robotiq_2f_85/robotiq_85_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
