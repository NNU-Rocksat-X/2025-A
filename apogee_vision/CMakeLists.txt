cmake_minimum_required(VERSION 3.0.2)
project(apogee_vision)

set(APOGEE_PLATFORM "Auxiliary")

if(${APOGEE_PLATFORM} STREQUAL "Jetson")
    set(PCL_DIR "/usr/lib/aarch64-linux-gnu/cmake/pcl/")
endif()
if(${APOGEE_PLATFORM} STREQUAL "Auxiliary")
    set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")
endif()

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
    COMPONENTS
    rospy
    roscpp
    std_msgs
    daedalus_msgs
    PCL
    pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        rospy
        roscpp
        pcl_ros
        daedalus_msgs
)

catkin_install_python(
    PROGRAMS
        scripts/depth_view
        scripts/view
        scripts/point_cloud_py
        scripts/cam_frame
        scripts/tests/view_orientation_state
        scripts/tests/orientation_axis
        scripts/tests/graph_angular_velocity
        nodes/remap_object_pose
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

##################################################
#                      LIBRARIES
##################################################
# TODO: change this to just collect all cpp files in src

add_library(eigen_util src/eigen_util.cpp)
add_dependencies(eigen_util ${catkin_EXPORTED_TARGETS})
target_link_libraries(eigen_util ${catkin_LIBRARIES}) 


add_library(tracker_lib src/tracker_lib.cpp)
add_dependencies(tracker_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracker_lib ${catkin_LIBRARIES})

add_library(velocity_filter src/trajectory_prediction/velocity_filter.cpp)
add_dependencies(velocity_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_filter ${catkin_LIBRARIES})

add_library(orientation_predictor src/trajectory_prediction/orientation_predictor.cpp)
add_dependencies(orientation_predictor ${catkin_EXPORTED_TARGETS})
target_link_libraries(orientation_predictor eigen_util ${CERES_LIBRARIES} ${catkin_LIBRARIES})

add_library(angular_filter src/trajectory_prediction/angular_filter.cpp)
add_dependencies(angular_filter  ${catkin_EXPORTED_TARGETS})
target_link_libraries(angular_filter eigen_util ${catkin_LIBRARIES})


##################################################
#                      EXECUTABLES
##################################################

add_executable(object_tracking src/object_tracking.cpp)
target_link_libraries(object_tracking tracker_lib ${catkin_LIBRARIES})
add_dependencies(object_tracking tracker_lib ${catkin_EXPORTED_TARGETS})


add_executable(object_tracking_gui src/object_tracking_gui.cpp)
target_link_libraries(object_tracking_gui tracker_lib ${catkin_LIBRARIES})
add_dependencies(object_tracking_gui tracker_lib ${catkin_EXPORTED_TARGETS})

add_executable(rviz_tracked_object src/rviz_tracked_object.cpp)
target_link_libraries(rviz_tracked_object ${catkin_LIBRARIES})
add_dependencies(rviz_tracked_object ${catkin_EXPORTED_TARGETS})

add_executable(point_cloud src/point_cloud.cpp)
target_link_libraries(point_cloud ${catkin_LIBRARIES})
add_dependencies(point_cloud ${catkin_EXPORTED_TARGETS})

add_executable(marsha_trajectory_predictor src/trajectory_prediction/marsha_trajectory_predictor.cpp)
target_link_libraries(marsha_trajectory_predictor ${catkin_LIBRARIES})
add_dependencies(marsha_trajectory_predictor ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_predictor src/trajectory_prediction/trajectory_predictor.cpp)
target_link_libraries(trajectory_predictor velocity_filter orientation_predictor ${catkin_LIBRARIES})
add_dependencies(trajectory_predictor ${catkin_EXPORTED_TARGETS})

