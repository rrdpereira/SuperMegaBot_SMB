#!/bin/bash
mkdir -p /home/$USER/3D_maps && cd $_
rosrun pcl_ros pointcloud_to_pcd input:=/rtabmap/cloud_map _prefix:=map3D_"$(date +%Y_%m_%d__%H_%M_%S_)"
