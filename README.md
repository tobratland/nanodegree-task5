# nanodegree-task5 - Home service robot

## TL:DR
This robot will pick up a object from one of five random places in my awesome new place(patent pending). It will deliver the object to a different spot in the appartment. 

## Try it out!
1. Clone the repository
2. Use the command `git submodule init` from the `src` folder
2. Use the command `git submodule update` from the `src` folder
3. Run one of the shellscripts in src/shellscripts(The home_service.sh is the script to run the complete package.)


![Home service robot](https://github.com/tobratland/nanodegree-task5/blob/main/images/home_service5.png)

## Technologies used
### Turtlebot
I use turtlebot as the simulated robot here. I also use turtlebot packages for simulating in gazebo. Turtlebot is a open source project, and its even possible to buy a kit and build a real turtlebot! The simulated turtlebot comes with several pre-built simulator environments, but in this project i use my own simulated environment in gazebo.

### Turtlebot Teleop package
Gives several interfaces to control the simulated turtlebot. The node that's used in this project is the keyboard_teleop node. It allows for input from the keyboard into a terminal, and uses the input to controll the robot. The `test_slam.sh` script uses this node. 

### Rviz
A way to visualize in 3d the robots way of viewing the world through viewing turtlebots different sensordata input.

### Gmapping package
This is a ros node that provides laser-based SLAM, which means Simultaneous Localization and mapping.
It receives laser and odometry data from the turtlebot simulation, and converts the data to a 2d map. To test this package run the `test_slam.sh` script. If you wish to save the generated map shown in rviz, use the command `rosrun map_saver map_saver`.

### Pick_Objects
My package for going to the spot the package is, will listen to marker topic to know where it should tell the move_base of turtlebot to move to. Listens to the `marker_position` topic to know where to ask turtlebot to move.   

### Add_Markers
This node is responsible for knowing where the current marker is, and picking a new marker position when the last one is reached. When turtlebot is close enough it will remove the marker and place a new one, and publish where the new one is. Uses the `amcl_pose` topic to know where the turtlebot is, and publishes marker position to both `visualization_marker` and `marker_position` topics. 

