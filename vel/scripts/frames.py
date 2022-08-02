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

from bosdyn.client.frame_helpers import get_odom_tform_body, add_edge_to_tree
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.math_helpers import Quat, SE3Pose
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped

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
    rospy.init_node('click')
    sdk = bosdyn.client.create_standard_sdk('VelodyneClient')
    robot = sdk.create_robot('192.168.80.3')
    robot.authenticate('user', 'wruzvkg4rce4')
    #bosdyn.client.util.authenticate(robot)
    robot.sync_with_directory()
    _robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    _robot_state_task = AsyncRobotState(_robot_state_client)
    _task_list = [_robot_state_task]
    _async_tasks = AsyncTasks(_task_list)

    update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    update_thread.daemon = True
    update_thread.start()

    r= rospy.Rate(1)
    frame_tree_edges={}
    while any(task.proto is None for task in _task_list):
        time.sleep(0.1)
        print('wait')
    odom_tform_start_prova=get_odom_tform_body(_robot_state_task.proto.kinematic_state.transforms_snapshot).to_proto()
    odom_tform_start=SE3Pose(odom_tform_start_prova.position.x,odom_tform_start_prova.position.y,odom_tform_start_prova.position.z,odom_tform_start_prova.rotation)
    print(odom_tform_start)
    while not rospy.is_shutdown():
        odom_tform_body_prova = get_odom_tform_body(_robot_state_task.proto.kinematic_state.transforms_snapshot).to_proto()
        odom_tform_body=SE3Pose(odom_tform_body_prova.position.x,odom_tform_body_prova.position.y,odom_tform_body_prova.position.z, odom_tform_body_prova.rotation)
        print("in frame odometry")
        print(odom_tform_body)
        start_tform_body=odom_tform_start.inverse()*odom_tform_body
        print("in frame start")
        print(start_tform_body)
        #frame_tree_edges=add_edge_to_tree(frame_tree_edges, start_tform_body, 'start', 'body')
        r.sleep()
    

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
