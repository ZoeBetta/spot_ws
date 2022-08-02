#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"


ros::Publisher tf_pub;
tf::TransformListener *tf_listener; 


void pointcloudcallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  35 {
       sensor_msgs::PointCloud2 pcl_out;
       tf_listener->waitForTransform("start", (*msg).header.frame_id, (*msg).header.stamp, ros::Duration(5.0));
  36   pcl_ros::transformPointCloud("start", *msg, pcl_out , *tf_listener);
       tf_pub.publish(pcl_out);
  37 }

int main(int argc, char** argv)
{
ros::init(argc, argv, "mapconversion");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("/points2", 1000, pointcloudcallback);
pcl_ros::transformPointCloud(fixedFrame, *cloud, *tcloud, tf_);
tf_pub = nh.advertise<sensor_msgs::PointCloud2> ("tf_points2", 1);

tf_listener    = new tf::TransformListener();

ros::spin();
}


