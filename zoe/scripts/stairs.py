#!/usr/bin/env python3

from __future__ import print_function

from __future__ import absolute_import

import argparse
import collections
import logging
import sys
import time
import threading
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import rospy
import tf

import bosdyn
import bosdyn.client
import bosdyn.client.util

from bosdyn.api import world_object_pb2
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks

from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b,
                                         get_vision_tform_body, get_odom_tform_body)
from bosdyn.client.world_object import WorldObjectClient


from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from zoe.msg import fiducial
from nav_msgs.msg import Odometry

from tf import transformations

LOGGER = logging.getLogger(__name__)
odom=Odometry()
def _update_thread(async_task):
    while True:
        async_task.update()
        time.sleep(0.01)

class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()

def clbkodom(msg):
    odom=msg
    #print(odom)


def main(argv):
    # The last argument should be the IP address of the robot. The app will use the directory to find
    # the velodyne and start getting data from it.
    rospy.init_node('stairs_detection')

    sdk = bosdyn.client.create_standard_sdk('Stairs')
    robot = sdk.create_robot('192.168.80.3')
    robot.authenticate('user', 'wruzvkg4rce4')
    #bosdyn.client.util.authenticate(robot)
    robot.sync_with_directory()

   # _point_cloud_client = robot.ensure_client('velodyne-point-cloud')
    _robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    _robot_state_task = AsyncRobotState(_robot_state_client)
    #_world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    _task_list = [_robot_state_task]
    _async_tasks = AsyncTasks(_task_list)

    update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    update_thread.daemon = True
    update_thread.start()
    #_world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    print('Connected.')
    # initialization of the publisher
    pub = rospy.Publisher('/fiducials', fiducial, queue_size=10)
    rospy.Subscriber('spot/odometry', Odometry, clbkodom)
    msg=fiducial()
    # initialization of tf
    listener = tf.TransformListener()
    body_tform_odom=[]
    #update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    #update_thread.daemon = True
    #update_thread.start()
   
    # Wait for the first responses.
    while not rospy.is_shutdown():
        #request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        #fiducial_objects = _world_object_client.list_world_objects(object_type=request_fiducials).world_objects
        try:
            (trans,rot) = listener.lookupTransform('/start', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #print(trans)
        #print(rot)
        euler=transformations.euler_from_quaternion(rot)
        trMat=np.array([ [np.cos(euler[0])*np.cos(euler[1])*np.cos(euler[2])-np.sin(euler[0])*np.sin(euler[2]), -np.cos(euler[0])*np.cos(euler[1])*np.sin(euler[2])-np.sin(euler[0])*np.cos(euler[2]), np.cos(euler[0])*np.sin(euler[1]), trans[0]],
            [np.sin(euler[0])*np.cos(euler[1])*np.cos(euler[2])-np.cos(euler[0])*np.sin(euler[2]), -np.sin(euler[0])*np.cos(euler[1])*np.sin(euler[2])+np.cos(euler[0])*np.cos(euler[2]), np.sin(euler[0])*np.sin(euler[1]), trans[1]],
            [-np.sin(euler[1])*np.cos(euler[2]), np.sin(euler[1])*np.sin(euler[2]), np.cos(euler[1]), trans[2]],
            [0,0,0,1]
            ]
            )
        body_tform_odom=[odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z,1]
        pos_tform_start= np.matmul(trMat,body_tform_odom)
        print(pos_tform_start)


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
