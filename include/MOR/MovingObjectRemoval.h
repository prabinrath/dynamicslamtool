#include "MOR/IncludeAll.h"

//for testing purpose only
ros::Publisher pub,marker_pub;

struct MovingObjectDetectionCloud
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,cluster_collection;
	pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_collection;
	vector<pcl::PointIndices> cluster_indices;
	vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
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
	public:
		bool volumeConstraint(pcl::PointCloud<pcl::PointXYZI>::Ptr fp, pcl::PointCloud<pcl::PointXYZI>::Ptr fc,double threshold);
		void calculateCorrespondenceCentroid(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr mp,double delta);
		vector<double> getPointDistanceEstimateVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::CorrespondencesPtr mp);
		vector<long> getClusterPointcloudChangeVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::CorrespondencesPtr mp,float resolution);
};

struct MovingObjectCentroid
{
	pcl::PointXYZ centroid;
	int confidence,max_confidence;

	MovingObjectCentroid(pcl::PointXYZ c,int n_good):centroid(c),confidence(n_good),max_confidence(n_good){}
	bool decreaseConfidence(){confidence--;if(confidence==0){return true;}return false;}
	void increaseConfidence(){if(confidence<max_confidence){confidence++;}}
};

class MovingObjectRemoval
{
	vector<MovingObjectCentroid> mo_vec;
	deque<pcl::CorrespondencesPtr> corrs_vec;
	deque<vector<bool>> res_vec;
	boost::shared_ptr<MovingObjectDetectionCloud> ca,cb;
	boost::shared_ptr<MovingObjectDetectionMethods> mth;
	int moving_confidence,static_confidence;
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;

	int recurseFindClusterChain(int col,int track);
	void checkMovingClusterChain(pcl::CorrespondencesPtr mp,vector<bool> &res_ca,vector<bool> &res_cb);
	void pushCentroid(pcl::PointXYZ pt);
	public:
		sensor_msgs::PointCloud2 output;
		MovingObjectRemoval(int n_bad,int n_good);
		void pushRawCloudAndPose(pcl::PCLPointCloud2 &cloud,geometry_msgs::Pose pose);
		bool filterCloud(string f_id);
};