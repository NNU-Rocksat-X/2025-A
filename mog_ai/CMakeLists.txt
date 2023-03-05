cmake_minimum_required(VERSION 3.0.2)
project(mog_ai)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
        roscpp
        std_msgs
        daedalus_msgs
        marsha_msgs
        geometry_msgs
)


catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        rospy
        roscpp
        daedalus_msgs
        marsha_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS
    scripts/marsha_ai
    scripts/reset_server
    scripts/grasp_data_generator
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(grasp_generator src/grasp_generator.cpp)
target_link_libraries(grasp_generator ${catkin_LIBRARIES})
add_dependencies(grasp_generator marsha_msgs_generate_messages_cpp)