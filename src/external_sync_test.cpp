#include "MOR/MovingObjectRemoval.h"

// ros::Publisher pub;

boost::shared_ptr<MovingObjectRemoval> mor;

void moving_object_test(const sensor_msgs::PointCloud2ConstPtr& input)
{
	clock_t begin_time = clock();
  std::cout<<"-----------------------------------------------------\n";
	pcl::PCLPointCloud2 cloud;
	pcl_conversions::toPCL(*input, cloud);

  geometry_msgs::Pose zero_pose;
  zero_pose.position.x = 0; zero_pose.position.y = 0; zero_pose.position.z = 0;
  zero_pose.orientation.x = 0; zero_pose.orientation.y = 0; zero_pose.orientation.z = 0; zero_pose.orientation.w = 1;

	mor->pushRawCloudAndPose(cloud,zero_pose);
	if(mor->filterCloud(cloud,"/filtered"))
	{
		//pub.publish(mor->output);
	}

  std::cout<<1000.0*(clock()-begin_time)/CLOCKS_PER_SEC<<std::endl;
  std::cout<<"-----------------------------------------------------\n";
}

int main (int argc, char** argv)
{
  #ifndef INTERNAL_SYNC 
  ros::init (argc, argv, "test_moving_object");
  ros::NodeHandle nh;
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 10);

  // message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/velodyne_points", 1);
  // message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/camera/odom/sample", 1);
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
  // sync.registerCallback(boost::bind(&moving_object_test, _1, _2));

  ros::Subscriber sub = nh.subscribe("evsts/clusters", 10, moving_object_test);

  std::string path = ros::package::getPath("dynamic_slam_tool");
  mor.reset(new MovingObjectRemoval(nh, path + "/config/MOR_config.txt",3,2));

  ros::spin();
  #endif
}