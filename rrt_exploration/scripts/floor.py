#!/usr/bin/env python3

import rospy
import tf
from numpy import array
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
from shapely.geometry import Point as pt
from shapely.geometry.polygon import Polygon
from shapely.geometry import  LineString
from zoe.msg import fiducial
from rrt_exploration.srv import Floor, FloorResponse
#________________________________________________________________________________
def handle_floor(req):
	global floor
	msg=FloorResponse()
	if (req.req==1):
		floor=req.new_floor
		msg.floor=rospy.wait_for_service('retrievefloor')
	else:
		msg.floor=floor
	return msg


def node():
	global floor
	rospy.init_node('floormanagement')
	floor=rospy.get_param('~init_floor')
	print(floor)
	s=rospy.Service('retrievefloor', Floor, handle_floor)
	rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass






















	
	
	
	
	
	
	
	
	
	
