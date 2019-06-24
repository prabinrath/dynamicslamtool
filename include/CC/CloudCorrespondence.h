#include "CC/IncludeAll.h"

struct CloudCorrespondence
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,cluster_collection;
	pcl::PointCloud<pcl::Normal>::Ptr nor;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr ld;
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::PointIndices desc_indices;
	std::vector<vector<double>> feature_bank;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
	static pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree;
	tf::Pose ps;
	bool init;
	CloudCorrespondence()
	{
		cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
		cluster_collection.reset(new pcl::PointCloud<pcl::PointXYZI>);
		nor.reset(new pcl::PointCloud<pcl::Normal>);
		ld.reset(new pcl::PointCloud<pcl::FPFHSignature33>);
		init = false;
	}
	void initCC();
	void filterPassThrough(float,float,float);
	void removePlaneSurface(float);
	void computeClusters(float,string);
	static bool cluster_condition(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance);
};

class CloudCorrespondenceMethods
{
	void swap_check_correspondence_ESFkdtree(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, map<int,pair<int,double>> &mp, double delta);
	void swap_check_correspondence_centroidKdtree(vector<vector<double>> &fp,vector<vector<double>> &fc, map<int,pair<int,double>> &mp, double delta);
	public:
		void calculate_correspondence_dtw(vector<vector<double>> &fp, vector<vector<double>> &fc, map<int,pair<int,double>> &mp,double delta);
		void calculate_correspondence_ESFkdtree(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, map<int,pair<int,double>> &mp,double delta);
		void calculate_correspondence_centroidKdtree(vector<vector<double>> &fp, vector<vector<double>> &fc, map<int,pair<int,double>> &mp,double delta);
};