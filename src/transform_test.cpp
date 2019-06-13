#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

using namespace std;

ros::Publisher pub;
sensor_msgs::PointCloud2 cloud_;
tf::Pose p1,p2;
int flag = false;

Eigen::Matrix4f getTransformFromPose(tf::Pose &p1,tf::Pose &p2)
{
	tf::Transform t = p1.inverseTimes(p2);
	//t=t.inverse();
	Eigen::Matrix4f m;
  	pcl_ros::transformAsMatrix(t,m);
  	cout<<m<<endl<<endl;
  	return m;
}

void transform_pc(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
	if(!flag)
	{
		tf::poseMsgToTF(odm->pose.pose,p1);
		cloud_ = *input;
		flag = true;
	}
	else
	{
		tf::poseMsgToTF(odm->pose.pose,p2);
		Eigen::Matrix4f m = getTransformFromPose(p1,p2);
  		sensor_msgs::PointCloud2Ptr tranformedPC(new sensor_msgs::PointCloud2());
  		pcl_ros::transformPointCloud(m,cloud_,*tranformedPC);
  		tranformedPC->header.stamp = input->header.stamp;
  		tranformedPC->header.frame_id = "/previous";
  		pub.publish(*tranformedPC);
  		cloud_ = *input;
  		cloud_.header.frame_id = "/current";
  		pub.publish(cloud_);
  		p1 = p2;
	}
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "transform_pc");
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/camera/odom/sample", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
  sync.registerCallback(boost::bind(&transform_pc, _1, _2));

  ros::spin();
}