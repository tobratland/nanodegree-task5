export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find my_robot)/worlds/NewAppartment.world


terminator -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
terminator -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/robond/workspace/nanodegree-task5/src/maps/map.yaml" &
sleep 5
terminator -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3
terminator -e "rosrun pick_objects pick_objects" &
sleep 3
terminator -e "rosrun add_markers add_markers"