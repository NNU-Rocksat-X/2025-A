cmake_minimum_required(VERSION 3.0.2)
project(mog_ai)

find_package(catkin REQUIRED
    COMPONENTS
        message_generation
        rospy
        roscpp
        std_msgs
        daedalus_msgs
        marsha_msgs
        geometry_msgs
)


catkin_python_setup()

add_service_files(
    FILES 
    GenerateGrasp.srv
)

generate_messages(
    DEPENDENCIES 
    std_msgs
    geometry_msgs
)

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
    nodes/marsha_ai
    nodes/reset_server
    nodes/catch_ai
    nodes/vae_server
    scripts/grasp_data_generator
    scripts/test_vae_grasps
    scripts/spawn_tensorboard
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(grasp_generator src/grasp_generator.cpp)
target_link_libraries(grasp_generator ${catkin_LIBRARIES})
add_dependencies(grasp_generator marsha_msgs_generate_messages_cpp)