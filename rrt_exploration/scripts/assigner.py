#!/usr/bin/env python3

#--------Include modules---------------
from asyncio import start_unix_server
from copy import copy
import rospy
from functions import IsStairs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from rrt_exploration.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount, FloorGain
from numpy.linalg import norm
from rrt_exploration.srv import Floor, FloorRequest

# Subscribers' callbacks------------------------------
mapData=[OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid()]
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]
def callBack(data):
	global frontiers
	frontiers=[]
	#rospy.loginfo("frontiers received")
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
	global mapData,floor
	rospy.wait_for_service('retrievefloor')
	req=FloorRequest()
	req.req=0
	fl=floor_service(req)
	floor=fl.floor
	mapData[floor]=data
    #rospy.loginfo("map received")
# Node----------------------------------------------

def StaircallBack(data):
	global frontier_stairs, floor_service
	frontier_stairs=[]
	for point in data.points:
		frontier_stairs.append(array([point.x,point.y,point.z]))

def node():
	global frontiers,mapData,global1,global2,global3,globalmaps, frontier_stairs,floor,floor_service
	rospy.init_node('assigner', anonymous=False)
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/grid_map')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
	n_robots = rospy.get_param('~n_robots',1)
	namespace = rospy.get_param('~namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count',1)
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
	rateHz = rospy.get_param('~rate',100)
	distance_weigth=rospy.get_param('~distance_weight')
	floor_weight=rospy.get_param('~floor_weight')
	maxexplored=rospy.get_param('~maxexplored')
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	floor_service=rospy.ServiceProxy('retrievefloor', Floor)
	rospy.wait_for_service('retrievefloor')
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber('/filtered_points', PointArray, callBack)
	rospy.Subscriber('/stair_points', PointArray, StaircallBack)
	
	
#---------------------------------------------------------------------------------------------------------------
		
# wait if no frontier is received yet 
	while len(frontiers)<1:
		#rospy.loginfo("frontiers not received")
		pass
	centroids=copy(frontiers)	
#wait if map is not received yet
	req=FloorRequest()
	req.req=0
	fl=floor_service(req)
	floor=fl.floor
	while (len(mapData[floor].data)<1):
		rospy.loginfo("map not received")
		pass

	robots=[]
	print("before assigning namespace")
	if len(namespace)>0:
		for i in range(0,n_robots):
			print(robot(namespace))
			robots.append(robot(namespace))
	elif len(namespace)==0:
			print(robot(namespace))
			robots.append(robot(namespace))
	print("before for loop")
	for i in range(0,n_robots):
		robots[i].sendGoal(robots[i].getPosition())
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		rospy.loginfo("inside main")
		req=FloorRequest()
		req.req=0
		fl=floor_service(req)
		floor=fl.floor
		centroids=copy(frontiers)	
		stairs=copy(frontier_stairs)	
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain=[]
		stairsinfoGain=[]
		for s in range(0,len(stairs)):
			# crea la funzione che inizializza i gain exploration per le frontiere scale
			# per ogni frontiera scale calcolo quale sarebbe il guadagno supposto del floor
			stairsinfoGain.append(FloorGain(mapData[int(stairs[s][2])],maxexplored))
			print(s)

		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData[floor],[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------			
#get number of available/busy robots - uses a function of the movebase node
		na=[] #available robots
		nb=[] #busy robots
		for i in range(0,n_robots):
			if (robots[i].getState()==1):
				nb.append(i)
			else:
				na.append(i)	
		#rospy.loginfo("available robots: "+str(na))	
#------------------------------------------------------------------------- 
#get dicount and update informationGain
		for i in nb+na:
			infoGain=discount(mapData[floor],robots[i].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------            
		revenue_record=[]
		centroid_record=[]
		id_record=[]
		floor_gain=FloorGain(mapData[floor],maxexplored)
		#print("floor gain for centroids: "+str(floor_gain))
		for ir in na:
			for ip in range(0,len(centroids)):
				# aggiungi il guadagno per il totale del piano
				cost=norm(robots[ir].getPosition()-centroids[ip])		
				threshold=1
				information_gain=infoGain[ip]
				if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
					information_gain*=hysteresis_gain
				# aggiungi weight per il cost per raggiungere e aggiungi il weight
				revenue=information_gain*info_multiplier-cost*distance_weigth+floor_gain*floor_weight
				revenue_record.append(revenue)
				centroid_record.append(centroids[ip])
				id_record.append(ir)
			for si in range(0, len(stairs)):
				cost=norm(robots[ir].getPosition()-array([stairs[si].x,stairs[si].y]))
				information_gain=stairsinfoGain[si]
				revenue=information_gain*floor_weight-cost*distance_weigth
				revenue_record.append(revenue)
				centroid_record.append(stairs[si])
				id_record.append(ir)

		
		if len(na)<1:
			revenue_record=[]
			centroid_record=[]
			id_record=[]
			floor_gain=FloorGain(mapData[floor],maxexplored)
			for ir in nb:
				for ip in range(0,len(centroids)):
					cost=norm(robots[ir].getPosition()-centroids[ip])		
					threshold=1
					information_gain=infoGain[ip]
					if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
						information_gain*=hysteresis_gain
				
					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
						information_gain=informationGain(mapData[floor],[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain

					revenue=information_gain*info_multiplier-cost*distance_weigth+floor_gain*floor_weight
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)
				for si in range(0, len(stairs)):
					cost=norm(robots[ir].getPosition()-array([stairs[si][0],stairs[si][1]]))
					information_gain=stairsinfoGain[si]
					if (norm(robots[ir].getPosition()-array([stairs[si][0],stairs[si][1]]))<=hysteresis_radius):
						information_gain*=hysteresis_gain
					if ((norm(array([stairs[si][0],stairs[si][1]])-robots[ir].assigned_point))<hysteresis_radius):
						information_gain*=hysteresis_gain
					revenue=information_gain*floor_weight-cost*distance_weigth
					revenue_record.append(revenue)
					centroid_record.append(stairs[si])
					id_record.append(ir)
		
		rospy.loginfo("revenue record: "+str(revenue_record))	
		rospy.loginfo("centroid record: "+str(centroid_record))	
		rospy.loginfo("robot IDs record: "+str(id_record))	
		
#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
			rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))	
			rospy.sleep(delay_after_assignement)
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
