#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
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
	tf::Transform t;
  double roll1, pitch1, yaw1, roll2, pitch2, yaw2;
  float linearposx1,linearposy1,linearposz1,linearposx2,linearposy2,linearposz2;
  boost::shared_ptr<tf::Quaternion> qtn;
  tf::Matrix3x3 mat;

  linearposx1 = p1.getOrigin().getX();
  linearposy1 = p1.getOrigin().getY();
  linearposz1 = p1.getOrigin().getZ();
  qtn.reset(new tf::Quaternion(p1.getRotation().getX(), p1.getRotation().getY(), p1.getRotation().getZ(), p1.getRotation().getW()));
  mat.setRotation(*qtn);
  mat.getRPY(roll1, pitch1, yaw1);

  linearposx2 = p2.getOrigin().getX();
  linearposy2 = p2.getOrigin().getY();
  linearposz2 = p2.getOrigin().getZ();
  qtn.reset(new tf::Quaternion(p2.getRotation().getX(), p2.getRotation().getY(), p2.getRotation().getZ(), p2.getRotation().getW()));
  mat.setRotation(*qtn);
  mat.getRPY(roll2, pitch2, yaw2);

  qtn->setRPY(roll1-roll2,pitch1-pitch2,yaw1-yaw2);
  tf::Vector3 v(linearposx1-linearposx2,linearposy1-linearposy2,linearposz1-linearposz2);
  t.setOrigin(v);
  t.setRotation(*qtn);
  //t.inverse();
	Eigen::Matrix4f m;
  pcl_ros::transformAsMatrix(t,m);
  cout<<m<<endl<<endl;
  return m;
}

void transform_pc(const nav_msgs::OdometryConstPtr& odm)
{
	if(!flag)
	{
		tf::poseMsgToTF(odm->pose.pose,p1);
		flag = true;
	}
	else
	{
		tf::poseMsgToTF(odm->pose.pose,p2);
		Eigen::Matrix4f m = getTransformFromPose(p1,p2);
  	sensor_msgs::PointCloud2Ptr tranformedPC(new sensor_msgs::PointCloud2());
  	pcl_ros::transformPointCloud(m,cloud_,*tranformedPC);
  	tranformedPC->header.stamp = odm->header.stamp;
  	tranformedPC->header.frame_id = "/camera_odom_frame";
  	pub.publish(*tranformedPC);
  	//cloud_ = *tranformedPC;
  	//p1 = p2;
	}
}

int main (int argc, char** argv)
{
  pcl::io::loadPCDFile(argv[1],cloud_);
  ros::init (argc, argv, "transform_pc");
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  ros::Subscriber sub = nh.subscribe ("/camera/odom/sample", 1, transform_pc);
  ros::spin();
}