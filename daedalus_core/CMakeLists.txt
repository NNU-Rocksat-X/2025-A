cmake_minimum_required(VERSION 2.8.3)
project(daedalus_core)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS

)

catkin_install_python(
    PROGRAMS
    nodes/ARM1_pcs
    nodes/ARM2_pcs
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
