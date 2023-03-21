cmake_minimum_required(VERSION 2.8.3)
project(daedalus_core)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    daedalus_msgs 
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        rospy
        daedalus_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS
    nodes/detector_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
