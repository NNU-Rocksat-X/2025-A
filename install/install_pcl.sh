#!/bin/bash

mkdir ~/pkgs
cd ~/pkgs
wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.8.1.zip
unzip pcl-1.8.1.zip
cd pcl-pcl-1.8.1
mkdir build
cd build
cmake ..
make
