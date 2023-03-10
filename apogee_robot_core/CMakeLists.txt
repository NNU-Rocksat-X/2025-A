cmake_minimum_required(VERSION 3.0.2)
project(apogee_robot_core)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    std_msgs
    hardware_interface
    controller_manager
    daedalus_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
)

find_package(Eigen3 REQUIRED)


catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        hardware_interface
        controller_manager
        daedalus_msgs
        moveit_core
        moveit_ros_planning_interface
        moveit_msgs
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS
    scripts/tests/test_catch_planning
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_library(apogee_hardware_interface src/apogee_hardware_interface.cpp)
target_link_libraries(apogee_hardware_interface ${catkin_LIBRARIES})
add_dependencies(apogee_hardware_interface ${catkin_EXPORTED_TARGETS})

add_executable(moveit_interface src/moveit_interface.cpp)
target_link_libraries(moveit_interface ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(moveit_interface ${catkin_EXPORTED_TARGETS})

add_executable(hardware_interface_node src/hardware_interface_node.cpp)
target_link_libraries(hardware_interface_node apogee_hardware_interface ${catkin_LIBRARIES})
add_dependencies(hardware_interface_node ${catkin_EXPORTED_TARGETS})

add_executable(teensy_interface src/teensy_interface.cpp)
target_link_libraries(teensy_interface ${catkin_LIBRARIES})
add_dependencies(teensy_interface ${catkin_EXPORTED_TARGETS})