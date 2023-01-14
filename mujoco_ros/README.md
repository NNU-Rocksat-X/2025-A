# Mujoco Ros
A mujoco simulation implementation with a ros interface.

## Documentation
- [Known Issues](docs/known_issues.md)

## Installing

These instructions assume you are installing on linux Ubuntu.

### Install Mujoco

This repository has been tested with mujoco version 2.2.2

Run the following commands to install Mujoco:
   

    wget https://github.com/deepmind/mujoco/releases/download/2.2.2/mujoco-2.2.2-linux-x86_64.tar.gz

    sudo tar xzvf mujoco-2.2.2-linux-x86_64.tar.gz

You can also download it a different way here:

[Mujoco Download](https://github.com/deepmind/mujoco/releases)

The Mujoco documentation is available here:

[Mujoco Documentation](https://mujoco.readthedocs.io/en/latest/overview.html)

To install libglfw3 type this command:

`sudo apt install libglfw3`

`sudo apt install libglfw3-dev`

### Requirements

A robot description package can be used to simulate a robot. 
Currently the following packages are supported:
* [ARM2D2](https://github.com/Apogee-Robotics/arm2d2_description)






