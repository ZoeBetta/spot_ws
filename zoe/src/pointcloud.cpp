#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include <string>
#include <iostream>
#include <sstream>


ros::Publisher tf_pub;
tf::TransformListener *tf_listener; 
int actual_floor;

void pointcloudcallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
       sensor_msgs::PointCloud2 pcl_out;
	char s1[10]= "start";
 	//char s2[10]="number";
	//std::stringstream ss;
	//ss<<actual_floor;
	//ss>>s2;
	//std::strcat(s1,s2);
       tf_listener->waitForTransform(s1 ,(*msg).header.frame_id, (*msg).header.stamp, ros::Duration(5.0));
  pcl_ros::transformPointCloud(s1, *msg, pcl_out , *tf_listener);
       tf_pub.publish(pcl_out);
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "mapconversion");
ros::NodeHandle n;
ros::NodeHandle nh;
ros::NodeHandle n1;
//ros::param::get("~floor", actual_floor);

//std::cout << actual_floor << std::endl;

ros::Subscriber sub = n.subscribe("/points2", 1000, pointcloudcallback);
tf_pub = nh.advertise<sensor_msgs::PointCloud2> ("tf_points2", 1000);


tf_listener    = new tf::TransformListener();

ros::spin();
}



















