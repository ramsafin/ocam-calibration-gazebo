cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(ocam-calibration)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})