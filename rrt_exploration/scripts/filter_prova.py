#!/usr/bin/env python3

#--------Include modules---------------
from ast import Is
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped 
import tf
from numpy import array,vstack,delete
from functions import gridValue,informationGain,obstacles, IsStairs
from sklearn.cluster import MeanShift
from rrt_exploration.msg import PointArray
from shapely.geometry import Point as pt
from shapely.geometry.polygon import Polygon
from shapely.geometry import  LineString
from zoe.msg import fiducial
from rrt_exploration import Floor, FloorRequest

# Subscribers' callbacks------------------------------
mapData=[OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid(),OccupancyGrid()]
frontiers=[]
globalmaps=[]
detected_fiducials=[]
def callBack(data,args):
	global frontiers,min_distance
	transformedPoint=args[0].transformPoint(args[1],data)
	x=[array([transformedPoint.point.x,transformedPoint.point.y])]
	if len(frontiers)>0:
		frontiers=vstack((frontiers,x))
	else:
		frontiers=x
    

def mapCallBack(data):
	global mapData, floor
	req=FloorRequest()
	req.req=0
	fl=floor_service(req)
	floor=fl.floor
	mapData[floor]=data

def globalMap(data):
	global global1,globalmaps,litraIndx,namespace_init_count,n_robots
	global1=data
	if n_robots>1:
		indx=int(data._connection_header['topic'][litraIndx])-namespace_init_count
	elif n_robots==1:
		indx=0
	globalmaps[indx]=data

def fiducialCallBack(data):
	global detected_fiducials
	already_received=0
	for i in detected_fiducials:
		if (data.id == i.id):
			already_received=1
	if already_received==0:
		detected_fiducials.append(data)
		# print(detected_fiducials)
		# bubblesort algorithms to sort the fiducials in order
		for i in range(len(detected_fiducials)):
			already_sorted=True
			for j in range(len(detected_fiducials)-i-j):
				if detected_fiducials[j].id>detected_fiducials[j+1].id:
					detected_fiducials[j], detected_fiducials[j+1] = detected_fiducials[j+1], detected_fiducials[j]
					already_sorted= False
			if already_sorted:
				break
		#print(detected_fiducials)

# Node----------------------------------------------
def node():
	global frontiers,mapData,global1,global2,global3,globalmaps,litraIndx,n_robots,namespace_init_count, detected_fiducials,floor
	rospy.init_node('filter', anonymous=False)
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	threshold= rospy.get_param('~costmap_clearing_threshold',70)
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	goals_topic= rospy.get_param('~goals_topic','/detected_points')	
	n_robots = rospy.get_param('~n_robots',1)
	namespace = rospy.get_param('~namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count',1)
	rateHz = rospy.get_param('~rate',100)
	litraIndx=len(namespace)
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber('/fiducials', fiducial, fiducialCallBack)

#---------------------------------------------------------------------------------------------------------------
	for i in range(0,n_robots):
		globalmaps.append(OccupancyGrid())
	if len(namespace) > 0:
		for i in range(0,n_robots):
			rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, globalMap) 
	elif len(namespace)==0:
			rospy.Subscriber('/grid_map', OccupancyGrid, globalMap)
#wait if map is not received yet
	req=FloorRequest()
	req.req=0
	fl=floor_service(req)
	floor=fl.floor
	while (len(mapData[floor].data)<1):
		pass
#wait if any of robots' global costmap map is not received yet
	for i in range(0,n_robots):
		while (len(globalmaps[i].data)<1):
			pass
	global_frame="/"+mapData[floor].header.frame_id


	tfLisn=tf.TransformListener()
	if len(namespace) > 0:
		for i in range(0,n_robots):
			tfLisn.waitForTransform(global_frame[1:],'/base_link', rospy.Time(0),rospy.Duration(10.0))
	elif len(namespace)==0:
			tfLisn.waitForTransform(global_frame[1:], '/base_link', rospy.Time(0),rospy.Duration(10.0))
	
	rospy.Subscriber('detected_points', PointStamped, callback=callBack,callback_args=[tfLisn,global_frame[1:]])
	pub = rospy.Publisher('frontiers', Marker, queue_size=10)
	pub2 = rospy.Publisher('centroids', Marker, queue_size=10)
	filterpub = rospy.Publisher('filtered_points', PointArray, queue_size=10)
	stairspub=rospy.Publisher('stair_points', PointArray, queue_size=10)
	rospy.loginfo("the map and global costmaps are received")
	floor_service=rospy.ServiceProxy('retrievefloor', Floor)
	
	# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	
	
	points=Marker()
	points_clust=Marker()
#Set the frame ID and timestamp.  See the TF tutorials for information on these.
	points.header.frame_id= mapData[floor].header.frame_id
	points.header.stamp= rospy.Time.now()
	points.ns= "markers2"
	points.id = 0
	points.type = Marker.POINTS
#Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	points.action = Marker.ADD;
	points.pose.orientation.w = 1.0
	points.scale.x=0.2
	points.scale.y=0.2 
	points.color.r = 255.0/255.0
	points.color.g = 255.0/255.0
	points.color.b = 0.0/255.0
	points.color.a=1;
	points.lifetime = rospy.Duration();
	p=Point()
	p.z = 0;
	pp=[]
	pl=[]
	points_clust.header.frame_id= mapData[floor].header.frame_id
	points_clust.header.stamp= rospy.Time.now()
	points_clust.ns= "markers3"
	points_clust.id = 4
	points_clust.type = Marker.POINTS

#Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	points_clust.action = Marker.ADD;
	points_clust.pose.orientation.w = 1.0;
	points_clust.scale.x=0.2;
	points_clust.scale.y=0.2; 
	points_clust.color.r = 0.0/255.0
	points_clust.color.g = 255.0/255.0
	points_clust.color.b = 0.0/255.0
	points_clust.color.a=1;
	points_clust.lifetime = rospy.Duration();
	
	temppoint=PointStamped()
	temppoint.header.frame_id= mapData[floor].header.frame_id
	temppoint.header.stamp=rospy.Time(0)
	temppoint.point.z=0.0
	
	arraypoints=PointArray()
	arraystairs=PointArray()
	tempPoint=Point()
	tempPoint.z=0.0
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		req=FloorRequest()
		req.req=0
		fl=floor_service(req)
		floor=fl.floor
#-------------------------------------------------------------------------	
#Clustering frontier points
		arraystairs.points=[]
		centroids=[] 
		front=copy(frontiers)
		if len(front)>1:
			ms = MeanShift(bandwidth=0.3)   
			ms.fit(front)
			centroids= ms.cluster_centers_	 #centroids array is the centers of each cluster
			#rospy.loginfo(centroids)		

		#if there is only one frontier no need for clustering, i.e. centroids=frontiers
		if len(front)==1:
			centroids=front
		frontiers=copy(centroids)
#-------------------------------------------------------------------------	
#clearing old frontiers  
      
		z=0
		while z<len(centroids):
			cond=False
			temppoint.point.x=centroids[z][0]
			temppoint.point.y=centroids[z][1]
						
			for i in range(0,1):
				transformedPoint=tfLisn.transformPoint(globalmaps[i].header.frame_id,temppoint)
				x=array([transformedPoint.point.x,transformedPoint.point.y])
				cond=(gridValue(globalmaps[i],x)>threshold) or cond
				#rospy.loginfo(cond)
				rospy.loginfo((informationGain(mapData[floor],[centroids[z][0],centroids[z][1]],info_radius)))
			if (cond or (informationGain(mapData[floor],[centroids[z][0],centroids[z][1]],info_radius))<0.4 or (obstacles(mapData,[centroids[z][0],centroids[z][1]],info_radius))>2.5):
				centroids=delete(centroids, (z), axis=0)
				z=z-1
			#CONTROLLA SE SCALE
			new_floor=IsStairs(detected_fiducials,[centroids[z][0],centroids[z][1]],floor )
			if (IsStairs!=-1):
				print("it is on the stairs")
				tempPoint.x=centroids[z][0]
				tempPoint.y=centroids[z][1]
				tempPoint.z=new_floor
				arraystairs.points.append(copy(tempPoint))
				centroids=delete(centroids, (z), axis=0)
				z=z-1
			z+=1
		#rospy.loginfo(centroids)
#-------------------------------------------------------------------------
#publishing
		arraypoints.points=[]
		for i in centroids:
			tempPoint.x=i[0]
			tempPoint.y=i[1]
			arraypoints.points.append(copy(tempPoint))
		filterpub.publish(arraypoints)
		stairspub.publish(arraystairs)
		pp=[]
		for q in range(0,len(frontiers)):
			p.x=frontiers[q][0]
			p.y=frontiers[q][1]
			pp.append(copy(p))
		points.points=pp
		pp=[]
		#rospy.loginfo(len(centroids))
		for q in range(0,len(centroids)):
			p.x=centroids[q][0]
			p.y=centroids[q][1]
			pp.append(copy(p))
		points_clust.points=pp
		pub.publish(points)
		pub2.publish(points_clust) 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
	try:
		node()
	except rospy.ROSInterruptException:
		pass
 
 
 
 
