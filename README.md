# spot_ws
## how to run
In 5 different terminals  
roslaunch vel spot.launch  
roslaunch vel vel.launch  
roslaunch vel map.launch  
roslaunch vel move_base.launch  
roslaunch rrt_exploration single.launch  
When 3 messages appear  
rosrun vel clicked_point.py  


## what are the nodes called:
roslaunch vel spot.launch    
it launches rviz and all the bosdyn services

roslaunch vel vel.launch  
it launches the frames.py node that creates the frame start in the position the robot is in. the node vel2.py that reads the data from the sensors and return the pointcloud2. and the node pointcloudconv that converts the frame for the pointcloud from the frame odom (where the robot has been turned on) to the frame start.

roslaunch vel map.launch   
the node that creates the map. 

roslaunch vel move_base.launch   
the node that implements the path planning and path following

roslaunch rrt_exploration single.launch    
the exploration algorithm

rosrun vel clicked_point.py    
the area tp explore, it is a square of 30 meters from the start frame with that orientation

## TODO
migliorare il path planning
come decidere cosa e' scale, il prof ha detto fiducials bello

