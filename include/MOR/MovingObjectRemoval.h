#include "MOR/IncludeAll.h"
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;

struct MovingObjectDetectionCloud
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,cluster_collection;
	pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_collection;
	std::vector<pcl::PointIndices> cluster_indices;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
	std::vector<bool> detection_results;
	tf::Pose ps;
	bool init;

	MovingObjectDetectionCloud()
	{
		cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
		cluster_collection.reset(new pcl::PointCloud<pcl::PointXYZI>);
		centroid_collection.reset(new pcl::PointCloud<pcl::PointXYZ>);
		init = false;
	}
	void groundPlaneRemoval(float,float,float);
	void groundPlaneRemoval();
	void computeClusters(float,std::string);
};

class MovingObjectDetectionMethods
{
	public:
		bool volumeConstraint(pcl::PointCloud<pcl::PointXYZI>::Ptr fp, pcl::PointCloud<pcl::PointXYZI>::Ptr fc,double threshold);
		void calculateCorrespondenceCentroid(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr mp,double delta);
		std::vector<double> getPointDistanceEstimateVector(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::CorrespondencesPtr mp);
		std::vector<long> getClusterPointcloudChangeVector(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::CorrespondencesPtr mp,float resolution);
};

struct MovingObjectCentroid
{
	std::vector<pcl::PointXYZ> centroids;
	int confidence,max_confidence;

	MovingObjectCentroid(std::vector<pcl::PointXYZ> c,int n_good):centroids(c),confidence(n_good),max_confidence(n_good){}
	bool decreaseConfidence(){confidence--;if(confidence==0){return true;}return false;}
	void increaseConfidence(){if(confidence<max_confidence){confidence++;}}
};

class MovingObjectRemoval
{
	std::vector<MovingObjectCentroid> mo_vec;
	std::deque<pcl::CorrespondencesPtr> corrs_vec;
	std::deque<std::vector<bool>> res_vec;
	std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> centroid_track;
	boost::shared_ptr<MovingObjectDetectionCloud> ca,cb;
	boost::shared_ptr<MovingObjectDetectionMethods> mth;
	int moving_confidence,static_confidence;
	pcl::KdTreeFLANN<pcl::PointXYZ> xyz_tree;

	ros::NodeHandle& nh;
	#ifdef VISUALIZE
	ros::Publisher pub,marker_pub;
	#endif
	#ifdef INTERNAL_SYNC
	message_filters::Subscriber<sensor_msgs::PointCloud2> raw_pc_sub;
	message_filters::Subscriber<nav_msgs::Odometry> raw_odom_sub;
	boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_process;
	#endif

	message_filters::Subscriber<sensor_msgs::PointCloud2> map_pc_sub;
  	message_filters::Subscriber<nav_msgs::Odometry> map_odom_sub;
  	boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry>> sync_update;

  	void mapUpdateSubscriber(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm);
	void movingCloudObjectSubscriber(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm);
	int recurseFindClusterChain(int col,int track,std::vector<pcl::PointXYZ> &tv);
	void checkMovingClusterChain(pcl::CorrespondencesPtr mp,std::vector<bool> &res_ca,std::vector<bool> &res_cb,pcl::PointCloud<pcl::PointXYZ>::Ptr ca_cc, pcl::PointCloud<pcl::PointXYZ>::Ptr cb_cc);
	void pushCentroid(std::vector<pcl::PointXYZ> &tv);
	public:
		sensor_msgs::PointCloud2 output;
		MovingObjectRemoval(ros::NodeHandle _nh,std::string config_path,int n_bad,int n_good);
		void pushRawCloudAndPose(pcl::PCLPointCloud2 &cloud,geometry_msgs::Pose pose);
		bool filterCloud(pcl::PCLPointCloud2 &cloud,std::string f_id);
};