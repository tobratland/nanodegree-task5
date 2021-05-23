#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find my_robot)/worlds/NewAppartment.world

terminator -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
terminator -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
terminator -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &