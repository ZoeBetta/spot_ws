# spot_ws
## how to run
In 5 different terminals  
roslaunch vel spot.launch  
roslaunch vel map.launch  
roslaunch vel move_base.launch  
roslaunch rrt_exploration single.launch  
When 3 messages appear  
rosrun vel clicked_point.py  

## TODO
modify the frontier filter in order to delete stupid frontiers, like the ones with a lot of obstacles around, i prefer the robot not stuck that doesn't explore everything
