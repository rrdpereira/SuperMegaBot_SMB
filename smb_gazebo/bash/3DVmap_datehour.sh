#!/bin/bash
mkdir -p /home/$USER/3DV_maps && cd $_
rosrun pcl_ros pointcloud_to_pcd input:=/rslidar_points _prefix:=map3D_"$(date +%Y_%m_%d__%H_%M_%S_)"
