#!/bin/bash
mkdir -p /home/$USER/z_maps && cd $_
rosrun map_server map_saver -f mob_rob_"$(date +%Y_%m_%d__%H_%M_%S)"