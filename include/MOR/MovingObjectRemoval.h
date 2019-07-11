#include "MOR/IncludeAll.h"

//for testing purpose only
ros::Publisher pub,marker_pub;

struct MovingObjectDetectionCloud
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,cluster_collection;
	pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_collection;
	vector<pcl::PointIndices> cluster_indices;
	vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
	static pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree;
	vector<bool> detection_results;
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
	void computeClusters(float,string);
	static bool cluster_condition(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance);
};

class MovingObjectDetectionMethods
{
	bool volumeConstraint(pcl::PointCloud<pcl::PointXYZI>::Ptr fp, pcl::PointCloud<pcl::PointXYZI>::Ptr fc,double threshold);
	public:
		void calculateCorrespondenceCentroid(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr mp,double delta);
		vector<double> getPointDistanceEstimateVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::CorrespondencesPtr mp);
		vector<long> getClusterPointcloudChangeVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::CorrespondencesPtr mp,float resolution);
};

struct MOCentroid
{
	pcl::PointXYZ centroid;
	int confidence,max_confidence;

	MOCentroid(pcl::PointXYZ c,int n_good):centroid(c),confidence(n_good),max_confidence(n_good){}
	bool decreaseConfidence(){confidence--;if(confidence==0){return false;}return true;}
	void increaseConfidence(){if(confidence<max_confidence){confidence++;}}
};

class MovingObjectRemoval
{
	vector<MOCentroid> mo_vec;
	vector<pcl::CorrespondencesPtr> corrs_vec;
	vector<vector<bool>> res_vec;
	boost::shared_ptr<MovingObjectDetectionCloud> ca,cb;
	boost::shared_ptr<MovingObjectDetectionMethods> mth;
	int moving_confidence,static_confidence;
	sensor_msgs::PointCloud2 output;

	bool recurseFindClusterChain(int col,int track);
	void checkMovingClusterChain(pcl::CorrespondencesPtr mp,vector<int> res_ca,vector<int> res_cb);
	public:
		MovingObjectRemoval(int n_bad,int n_good);
		void pushRawCloudAndPose(pcl::PCLPointCloud2 &cloud,geometry_msgs::Pose pose);
};