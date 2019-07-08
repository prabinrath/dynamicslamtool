#include "MOD/MovingObjectDetection.h"

extern visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster, int id, string f_id, string ns="bounding_box", float r=0.5, float g=0.5, float b=0.5);
extern visualization_msgs::Marker mark_direction(double x,double y,double z,int id, string f_id, string ns="direction_vector", float r=0.5, float g=0.5, float b=0.5);
extern ros::Publisher pub,marker_pub;

boost::shared_ptr<MovingObjectDetection> ca,cb;
boost::shared_ptr<MovingObjectDetectionMethods> mth;

void moving_object_test(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
	sensor_msgs::PointCloud2 output;
  	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  	pcl_conversions::toPCL(*input, *cloud);
  
  	ca = cb;
  	cb.reset(new MovingObjectDetection());
  	pcl::fromPCLPointCloud2(*cloud, *(cb->cloud));

  	tf::poseMsgToTF(odm->pose.pose,cb->ps);
  	cb->groundPlaneRemoval(4.0,4.0,5.0);
	cb->init = true;

	if(ca->init  == true && cb->init == true)
	{
		clock_t begin_time = clock();
		tf::Transform t = (cb->ps).inverseTimes(ca->ps);

    	pcl::PointCloud<pcl::PointXYZI> temp = *ca->cloud;
	  	pcl_ros::transformPointCloud(temp,*ca->cloud,t);

	  	ca->computeClusters(0.11,"single_cluster");
		cb->computeClusters(0.11,"single_cluster");
		
		pcl::toPCLPointCloud2(*ca->cluster_collection,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "previous";
		pub.publish(output);
		pcl::toPCLPointCloud2(*cb->cluster_collection,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "current";
		pub.publish(output);
  		
	  	pcl::CorrespondencesPtr mp(new pcl::Correspondences());
	  	
	  	//cluster correspondence methods (Global)
	  	mth->calculateCorrespondenceCentroid(ca->centroid_collection,cb->centroid_collection,mp,0.1);
	  	
	  	//moving object detection methods (Local)
	  	//vector<double> param_vec = mth->getPointDistanceEstimateVector(ca->clusters,cb->clusters,mp);
	  	vector<long> param_vec = mth->getClusterPointcloudChangeVector(ca->clusters,cb->clusters,mp,0.1);

	  	float rs=0.4,gs=0.6,bs=0.8,rd=0.8,gd=0.1,bd=0.4;int id = 1;
	  	for(int j=0;j<mp->size();j++)
		{
			cout<<"{"<<(*mp)[j].index_query<<"->"<<(*mp)[j].index_match<<"} Fit Score: "<<(*mp)[j].distance<<" Moving_Score: "<<param_vec[id-1]<<endl;
			long threshold = (ca->clusters[(*mp)[j].index_query]->points.size()+cb->clusters[(*mp)[j].index_match]->points.size())/25;
			//double threshold = 0.15;
			if(param_vec[id-1]>threshold)
			{
				marker_pub.publish(mark_cluster(ca->clusters[(*mp)[j].index_query],id,"previous","bounding_box",rd,gd,bd));
				marker_pub.publish(mark_cluster(cb->clusters[(*mp)[j].index_match],id,"current","bounding_box",rd,gd,bd));
			}
			else
			{
				marker_pub.publish(mark_cluster(ca->clusters[(*mp)[j].index_query],id,"previous","bounding_box",rs,gs,bs));
				marker_pub.publish(mark_cluster(cb->clusters[(*mp)[j].index_match],id,"current","bounding_box",rs,gs,bs));
			}
			id++;
		}

		cout<<"-----------------------------------------------------\n";
		cout<<1000.0*(clock()-begin_time)/CLOCKS_PER_SEC<<endl;
	}
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "test_moving_object");
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 10);
  marker_pub = nh.advertise<visualization_msgs::Marker>("bbox", 10);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/camera/odom/sample", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
  sync.registerCallback(boost::bind(&moving_object_test, _1, _2));

  ca.reset(new MovingObjectDetection());
  cb.reset(new MovingObjectDetection());
  mth.reset(new MovingObjectDetectionMethods());

  ros::spin();
}