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
#________________________________________________________________________________
class robot:
	goal = MoveBaseGoal()
	start = PoseStamped()
	end = PoseStamped()
	
	def __init__(self,name):
		self.assigned_point=[]
		self.name=name
		self.global_frame=rospy.get_param('~global_frame','start')
		self.listener=tf.TransformListener()
		self.listener.waitForTransform(self.global_frame, '/base_link', rospy.Time(0),rospy.Duration(10.0))
		cond=0;	
		while cond==0:	
			try:
				(trans,rot) = self.listener.lookupTransform(self.global_frame, '/base_link', rospy.Time(0))
				cond=1
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				cond==0
		self.position=array([trans[0],trans[1]])		
		self.assigned_point=self.position
		self.client=actionlib.SimpleActionClient('/move_base', MoveBaseAction)
		self.client.wait_for_server()
		robot.goal.target_pose.header.frame_id='map'
		robot.goal.target_pose.header.stamp=rospy.Time.now()
		rospy.wait_for_service('/move_base/make_plan')
		self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
		robot.start.header.frame_id=self.global_frame
		robot.end.header.frame_id=self.global_frame

	def getPosition(self):
		cond=0;	
		while cond==0:	
			try:
				(trans,rot) = self.listener.lookupTransform(self.global_frame,'/base_link', rospy.Time(0))
				cond=1
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				cond==0
		self.position=array([trans[0],trans[1]])
		return self.position
		
	def sendGoal(self,point):
		robot.goal.target_pose.pose.position.x=point[0]
		robot.goal.target_pose.pose.position.y=point[1]
		robot.goal.target_pose.pose.orientation.w = 1.0
		self.client.send_goal(robot.goal)
		self.assigned_point=array([point[0],point[1]])
	
	def cancelGoal(self):
		self.client.cancel_goal()
		self.assigned_point=self.getPosition()
	
	def getState(self):
		return self.client.get_state()
	def getGoal(self):
		return self.assigned_point
		
	def makePlan(self,start,end):
		robot.start.pose.position.x=start[0]
		robot.start.pose.position.y=start[1]
		robot.end.pose.position.x=end[0]
		robot.end.pose.position.y=end[1]
		start=self.listener.transformPose('/grid_map', robot.start)
		end=self.listener.transformPose('/grid_map', robot.end)
		plan=self.make_plan(start = start, goal = end, tolerance = 0.0)
		return plan.plan.poses	
#________________________________________________________________________________

def index_of_point(mapData,Xp):
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y
	width=mapData.info.width
	Data=mapData.data
	index=int(	(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) ))
	return index
	
def point_of_index(mapData,i):
	y=mapData.info.origin.position.y+(i//mapData.info.width)*mapData.info.resolution
	x=mapData.info.origin.position.x+(i%mapData.info.width)*mapData.info.resolution#(i-(i/mapData.info.width)*(mapData.info.width))*mapData.info.resolution
	#rospy.loginfo(mapData.info.origin.position.y)
	#rospy.loginfo(mapData.info.width)
	return array([x,y])
#________________________________________________________________________________
# it calculates the number of unknown cells near the frontiers and returns the approximate values in square meters that will be discovered
# 		

def informationGain(mapData,point,r):
	infoGain=0;
	index=index_of_point(mapData,point)
	r_region=int(r/mapData.info.resolution)
	init_index=index-r_region*(mapData.info.width+1)
	# for each row close to point ( close in the meaning of radius of infogain) 
	for n in range(0,2*r_region+1):
		# find the start and finish index to explore
		start=n*mapData.info.width+init_index
		end=start+2*r_region
		limit=((start/mapData.info.width)+2)*mapData.info.width
		# for each index close in that line
		for i in range(start,end+1):
			# if the index is positive, not too far away and inside the map
			if (i>=0 and i<limit and i<len(mapData.data)):
				# if  the cell has to be discovered and if it is close enough in meters increase the expected information gain
				if(mapData.data[i]==-1 and norm(array(point)-point_of_index(mapData,i))<=r):
					infoGain+=1
			#rospy.loginfo(norm(array(point)-point_of_index(mapData,i)))
	#rospy.loginfo(mapData.info.resolution)
	#rospy.loginfo(infoGain)
	# return the information gain in how many meters do you expect to discover
	return infoGain*(mapData.info.resolution**2)

#________________________________________________________________________________	

def FloorGain(mapData, maxmeters):
	explored=0
	for i in range(0,len(mapData.data)):
		if mapData.data[i]!=-1:
			explored+=1
	return (maxmeters-explored*(mapData.info.resolution**2))

#________________________________________________________________________________		

def obstacles(mapData,point,r):
	infoGain=0;
	index=index_of_point(mapData,point)
	r_region=int(r/mapData.info.resolution)
	init_index=index-r_region*(mapData.info.width+1)	
	for n in range(0,2*r_region+1):
		start=n*mapData.info.width+init_index
		end=start+2*r_region
		limit=((start/mapData.info.width)+2)*mapData.info.width
		for i in range(start,end+1):
			if (i>=0 and i<limit and i<len(mapData.data)):
				if(mapData.data[i]>70 and norm(array(point)-point_of_index(mapData,i))<=r):
					infoGain+=1
			#rospy.loginfo(norm(array(point)-point_of_index(mapData,i)))
	#rospy.loginfo(mapData.info.resolution)
	#rospy.loginfo(infoGain)
	return infoGain*(mapData.info.resolution**2)
#_______________________________________________________________________________

def IsStairs(fiducial,point, floor):
	for i in fiducial:
		if i.floor==floor:
			for j in fiducial:
				if j.floor== floor:
					if ((i.id+1)==(j.id)):
						fiducial1=j
						fiducial2=i
						point1=pt(fiducial1.x,fiducial1.y)
						point2=pt(fiducial2.x, fiducial2.y)
						base_line=LineString([point1,point2])
						new_line=base_line.parallel_offset(4, 'left', 16, 2, 5.0)
						point3=pt(list(new_line.coords)[1])
						point4=pt(list(new_line.coords)[0])
						polygon= Polygon([point1, point2, point3,point4])
						frontier= pt(point[0],point[1])
						if polygon.contains(frontier):
							#frontier is stairs
							# return where the frontier leads to
							new_floor=i.id//10
							return new_floor
	return -1

#_______________________________________________________________________________

def discount(mapData,assigned_pt,centroids,infoGain,r):
	index=index_of_point(mapData,assigned_pt)
	r_region=int(r/mapData.info.resolution)
	init_index=index-r_region*(mapData.info.width+1)
	for n in range(0,2*r_region+1):
		start=n*mapData.info.width+init_index
		end=start+2*r_region
		limit=((start/mapData.info.width)+2)*mapData.info.width
		for i in range(start,end+1):
			if (i>=0 and i<limit and i<len(mapData.data)):
				for j in range(0,len(centroids)):
					current_pt=centroids[j]
					if(mapData.data[i]==-1 and norm(point_of_index(mapData,i)-current_pt)<=r and norm(point_of_index(mapData,i)-assigned_pt)<=r):
						infoGain[j]=infoGain[j] - 1*(mapData.info.resolution**2) #this should be modified, subtract the area of a cell, not 1
	return infoGain
#________________________________________________________________________________

def pathCost(path):
	if (len(path)>0):
		i=len(path)/2
		p1=array([path[i-1].pose.position.x,path[i-1].pose.position.y])
		p2=array([path[i].pose.position.x,path[i].pose.position.y])
		return norm(p1-p2)*(len(path)-1)
	else:
		return inf
#________________________________________________________________________________
		
def unvalid(mapData,pt):
	index=index_of_point(mapData,pt)
	r_region=5
	init_index=index-r_region*(mapData.info.width+1)
	for n in range(0,2*r_region+1):
		start=n*mapData.info.width+init_index
		end=start+2*r_region
		limit=((start/mapData.info.width)+2)*mapData.info.width	
		for i in range(start,end+1):	
			if (i>=0 and i<limit and i<len(mapData.data)):
				if(mapData.data[i]==1):
					return True
	return False
#________________________________________________________________________________
def Nearest(V,x):
	n=inf
	i=0
	for i in range(0,V.shape[0]):
		n1=norm(V[i,:]-x)
		if (n1<n):
			n=n1
			result=i    
	return result

#________________________________________________________________________________ 
def Nearest2(V,x):
	n=inf
	result=0
	for i in range(0,len(V)):
		n1=norm(V[i]-x)
		if (n1<n):
			n=n1
	return i
#________________________________________________________________________________
def gridValue(mapData,Xp):
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y
	width=mapData.info.width
	Data=mapData.data
 # returns grid value at "Xp" location
 #map data:  100 occupied      -1 unknown       0 free
	index=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) ) 
	if int(index) < len(Data):
		return Data[int(index)]
	else:
		return 100






















	
	
	
	
	
	
	
	
	
	
