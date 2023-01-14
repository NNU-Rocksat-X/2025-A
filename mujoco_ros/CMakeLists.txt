cmake_minimum_required(VERSION 3.0.2)
project(mujoco_ros)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

 
find_package(catkin REQUIRED
    COMPONENTS
    message_generation
    roscpp
    std_msgs
    daedalus_msgs
    geometry_msgs
)
find_package(mujoco REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

add_service_files(
    FILES 
    SetBody.srv
    ApplyForce.srv
)

generate_messages(
    DEPENDENCIES 
    std_msgs
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning_interface
        moveit_msgs
        daedalus_msgs
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${mujoco_INCLUDE_DIR} 
    ${GLFW_INCLUDE_DIRS}
)

add_library(glfw_dispatch src/glfw_dispatch.cc)
target_link_libraries(glfw_dispatch ${catkin_LIBRARIES})

add_library(ctrl_interface src/ctrl_interface.cpp)
target_link_libraries(ctrl_interface glfw_dispatch ${mujoco_LIBRARIES} ${catkin_LIBRARIES} ${GLFW_LIBRARIES})
add_dependencies(ctrl_interface ${catkin_EXPORTED_TARGETS})

add_library(physics src/physics.cpp)
target_link_libraries(physics ${mujoco_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(physics ${catkin_EXPORTED_TARGETS})

add_executable(ros_sim src/ros_sim.cpp)
target_link_libraries(ros_sim glfw_dispatch ctrl_interface physics ${mujoco_LIBRARIES} ${catkin_LIBRARIES} ${GLFW_LIBRARIES})
add_dependencies(ros_sim ${catkin_EXPORTED_TARGETS})

install(TARGETS ros_sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

 

