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


from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped


def main(argv):
    # The last argument should be the IP address of the robot. The app will use the directory to find
    # the velodyne and start getting data from it.
    rospy.init_node('click')
    pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
    point=PointStamped()
    time.sleep(2)
    point.header.frame_id='map'
    point.point.x=-30
    point.point.y=30
    pub.publish(point)
    time.sleep(2)
    point.point.x=-30
    point.point.y=-30
    pub.publish(point)
    time.sleep(2)
    point.point.x=30
    point.point.y=-30
    pub.publish(point)
    time.sleep(2)
    point.point.x=30
    point.point.y=30
    pub.publish(point)
    time.sleep(2)
    point.point.x=0
    point.point.y=0
    pub.publish(point)
    time.sleep(2)
    

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
