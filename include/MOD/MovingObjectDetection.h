#include "MOD/IncludeAll.h"

ros::Publisher pub,marker_pub;

struct MovingObjectDetection
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,cluster_collection;
	pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_collection;
	vector<pcl::PointIndices> cluster_indices;
	vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
	static pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree;
	tf::Pose ps;
	bool init;
	MovingObjectDetection()
	{
		cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
		cluster_collection.reset(new pcl::PointCloud<pcl::PointXYZI>);
		centroid_collection.reset(new pcl::PointCloud<pcl::PointXYZ>);
		init = false;
	}
	void groundPlaneRemoval(float,float,float);
	void removePlaneSurface(float);
	void computeClusters(float,string);
	static bool cluster_condition(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance);
};

class MovingObjectDetectionMethods
{
	bool volumeConstraint(pcl::PointCloud<pcl::PointXYZI>::Ptr fp, pcl::PointCloud<pcl::PointXYZI>::Ptr fc,double threshold);
	public:
		void calculateCorrespondenceCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr mp,double delta);
		vector<double> getPointDistanceEstimateVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp);
		vector<long> getClusterPointcloudChangeVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp,float resolution);
};