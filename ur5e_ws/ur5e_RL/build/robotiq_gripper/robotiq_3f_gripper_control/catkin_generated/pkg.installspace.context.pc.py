# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_manager;diagnostic_updater;dynamic_reconfigure;hardware_interface;robotiq_ethercat;roscpp;rospy;socketcan_interface;std_srvs;robotiq_3f_gripper_articulated_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lrobotiq_3f_gripper_control".split(';') if "-lrobotiq_3f_gripper_control" != "" else []
PROJECT_NAME = "robotiq_3f_gripper_control"
PROJECT_SPACE_DIR = "/home/ben/work/ur5e_RL/install"
PROJECT_VERSION = "1.0.0"
