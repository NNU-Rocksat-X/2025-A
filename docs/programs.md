# Programs


The following provides different commands to launch the robotic arms. Note that any combination of arguments can be used.

## Simulation

Launch mujoco simulator on an Auxiliary Platform with:
    
    roslaunch mujoco_ros sim.launch

Available Arguments:

`roslaunch mujoco_ros sim.launch env:=<Environment>`

Current <Environments> are `payload` and `wrench`

`roslaunch mujoco_ros sim.launch fixed_cam:=<true or false>`

`fixed_cam == true` means simulate from the depth cam's viewpoint


## Simulate with control

    roslaunch mujoco_ros arm2d2.launch

## Simulation with both arms
--------------------------
 
TODO

Simulation with RVIZ control interface
--------------------------
 
TODO: Marsha had the capability to deactivate rviz, this needs to be implemented for daedalus

    roslaunch arm2d2 simulate.launch launch_rviz:=true

RVIZ is a way to control the robot and see where it plans to move. Simply open the RVIZ window and drag the end effector marker to the desired position and then press plan and execute. The Gazebo simulated robot will move to this position and orientation, this works for the hardware robot as well. Individual joints can be controlled with RVIZ as well.

Simulation with the top plate
------------------------

TODO: Marsha had the capability to deactivate rviz, this needs to be implemented for daedalus
 
    roslaunch arm2d2 simulate.launch top_plate:=true

