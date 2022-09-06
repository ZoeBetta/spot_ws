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

from tf import transformations
from rrt_exploration import Floor, FloorRequest

LOGGER = logging.getLogger(__name__)

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

def main(argv):
    # The last argument should be the IP address of the robot. The app will use the directory to find
    # the velodyne and start getting data from it.
    rospy.init_node('fiducialdetection')

    sdk = bosdyn.client.create_standard_sdk('FiducialClient')
    robot = sdk.create_robot('192.168.80.3')
    robot.authenticate('user', 'wruzvkg4rce4')
    #bosdyn.client.util.authenticate(robot)
    robot.sync_with_directory()

   # _point_cloud_client = robot.ensure_client('velodyne-point-cloud')
    _robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    _robot_state_task = AsyncRobotState(_robot_state_client)
    _world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    _task_list = [_robot_state_task]
    _async_tasks = AsyncTasks(_task_list)

    update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    update_thread.daemon = True
    update_thread.start()
    #_world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    print('Connected.')
    # initialization of the publisher
    pub = rospy.Publisher('/fiducials', fiducial, queue_size=10)
    msg=fiducial()
    # initialization of tf
    listener = tf.TransformListener()
   
    #update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    #update_thread.daemon = True
    #update_thread.start()
   
    # Wait for the first responses.
    while not rospy.is_shutdown():
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = _world_object_client.list_world_objects(object_type=request_fiducials).world_objects
        try:
            (trans,rot) = listener.lookupTransform('/start', BODY_FRAME_NAME, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        floor_service=rospy.ServiceProxy('retrievefloor', Floor)
        req=FloorRequest()
        req.req=0
        fl=floor_service(req)
        floor=fl.floor
        #print(trans)
        #print(rot)
        for i in fiducial_objects:
            #print(fiducial_objects)
            #print(fiducial_objects.name)
            #print(i.apriltag_properties.frame_name_fiducial)
            # it sends the message with the name of the fiducial and its position
            msg.id=i.apriltag_properties.frame_name_fiducial
            #retrieve position in frame start
            vision_tform_fiducial = get_a_tform_b(i.transforms_snapshot, BODY_FRAME_NAME, i.apriltag_properties.frame_name_fiducial).to_proto()
            euler=transformations.euler_from_quaternion(rot)
            trMat=np.array([ [np.cos(euler[0])*np.cos(euler[1])*np.cos(euler[2])-np.sin(euler[0])*np.sin(euler[2]), -np.cos(euler[0])*np.cos(euler[1])*np.sin(euler[2])-np.sin(euler[0])*np.cos(euler[2]), np.cos(euler[0])*np.sin(euler[1]), trans[0]],
            [np.sin(euler[0])*np.cos(euler[1])*np.cos(euler[2])-np.cos(euler[0])*np.sin(euler[2]), -np.sin(euler[0])*np.cos(euler[1])*np.sin(euler[2])+np.cos(euler[0])*np.cos(euler[2]), np.sin(euler[0])*np.sin(euler[1]), trans[1]],
            [-np.sin(euler[1])*np.cos(euler[2]), np.sin(euler[1])*np.sin(euler[2]), np.cos(euler[1]), trans[2]],
            [0,0,0,1]
            ]
            )
            body_tform_fid=[vision_tform_fiducial.position.x, vision_tform_fiducial.position.y, vision_tform_fiducial.position.z, 1]
            start_tform_fiducial=np.matmul(trMat,body_tform_fid)
            #print(start_tform_fiducial)
            #print('transform')
            #print(vision_tform_fiducial.position)
            # do transformation matrix between vision and transform
            msg.x=start_tform_fiducial[0]
            msg.y=start_tform_fiducial[1]
            msg.z=start_tform_fiducial[2]
            msg.floor=floor
            pub.publish(msg)


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
