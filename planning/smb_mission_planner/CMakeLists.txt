cmake_minimum_required(VERSION 3.5)
project(smb_mission_planner)

find_package(catkin_simple REQUIRED)
catkin_python_setup()
catkin_simple(ALL_DEPS_REQUIRED)
cs_install()
cs_export()
