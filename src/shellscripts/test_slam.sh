#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find my_robot)/worlds/NewAppartment.world

sleep 5

terminator -e "source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
terminator -e "source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
terminator -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
terminator -e "source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"