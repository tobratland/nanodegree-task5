# nanodegree-task5 - Home service robot

## TL:DR
This robot will pick up a object from one of five random places in my awesome new place(patent pending). It will deliver the object to a different spot in the appartment. 


## Packages
# Turtlebot
I use turtlebot as the simulated robot here. I also use turtlebot packages for simulating in gazebo. 
# Slam gmapping
Slam gmapping is a slam package for ros, works great and gives a map of the environment.
# Pick_Objects
My package for going to the spot the package is, will listen to marker topic to know where it should tell the move_base of turtlebot to move to.
# Add_Markers
The package where the magic happen. Will publish where the marker is and listen for where turtlebot is. When turtlebot is close enough it will remove the marker and place a new one, and publish where the new one is. 

