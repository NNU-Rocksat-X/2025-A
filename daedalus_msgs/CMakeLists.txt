cmake_minimum_required(VERSION 3.0.2)
project(daedalus_msgs)

find_package(catkin REQUIRED 
    COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        genmsg
        actionlib_msgs
)

add_message_files(
    FILES
    TeensyMsg.msg
    OrientationState.msg
    RGBDFrame.msg
)
  
add_service_files(
    FILES
    GraspDetect.srv
    GetPos.srv
    MoveCmd.srv
    PositionCmd.srv
    PostureCmd.srv
    PlanGrasp.srv
    PredictPosition.srv
    ObjectObservation.srv
)


generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package(
    
)