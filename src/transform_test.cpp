#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <test_work/pc_odom.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

using namespace std;

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr ca,cb;

void transform_pc(const test_work::pc_odomConstPtr& msg)
{
  tf::Pose p1,p2;
  tf::poseMsgToTF(msg->odom.pose.pose,p1);
  tf::poseMsgToTF(msg->odom.pose.pose,p2);
  tf::Transform t = p1.inverseTimes(p2);
  Eigen::Matrix4f m;
  pcl_ros::transformAsMatrix(t,m);
  cout<<m<<endl<<endl;

  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(msg->cloud, *cloud);
  pcl::fromPCLPointCloud2(*cloud, *ca);
  pcl_ros::transformPointCloud(*ca,*cb,t);

}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "transform_pc");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/sync_packet", 1, transform_pc);
  ca.reset(new pcl::PointCloud<pcl::PointXYZI>);
  cb.reset(new pcl::PointCloud<pcl::PointXYZI>);
  ros::spin();
}
