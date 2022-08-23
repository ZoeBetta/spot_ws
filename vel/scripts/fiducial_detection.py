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

import bosdyn
import bosdyn.client
import bosdyn.client.util

from bosdyn.api import world_object_pb2
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks

from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.world_object import WorldObjectClient


from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

LOGGER = logging.getLogger(__name__)

def _update_thread(async_task):
    while True:
        async_task.update()
        time.sleep(0.01)


class AsyncPointCloud(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncPointCloud, self).__init__("point_clouds", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_point_cloud_from_sources_async(["velodyne-point-cloud"])


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
    #_point_cloud_task = AsyncPointCloud(_point_cloud_client)
    _robot_state_task = AsyncRobotState(_robot_state_client)
   # _task_list = [_point_cloud_task, _robot_state_task]
    #_async_tasks = AsyncTasks(_task_list)
    _world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    print('Connected.')

    #update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    #update_thread.daemon = True
    #update_thread.start()

    # Wait for the first responses.
    while not rospy.is_shutdown():

        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = _world_object_client.list_world_objects(object_type=request_fiducials).world_objects
        if len(fiducial_objects) > 0:
            fiducial=fiducial_objects[0]
            #print(fiducial_objects)
            #print(fiducial_objects.name)
            print(fiducial.apriltag_properties.frame_name_fiducial)


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
