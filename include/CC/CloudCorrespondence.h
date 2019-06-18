#include "CC/IncludeAll.h"

struct CloudCorrespondence
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,cluster_collection;
	std::vector<pcl::PointIndices> cluster_indices;
	std::vector<vector<double>> feature_bank;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
	static pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;
	tf::Pose ps;
	bool init;

	CloudCorrespondence()
	{
		cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
		init = false;
	}
	void initCC();
	void filterPassThrough(float,float,float);
	void removePlaneSurface(float);
	void computeClusters(float,string);
};

class CloudCorrespondenceMethods
{
	void swap_check_correspondence_VFHkdtree(vector<vector<double>> &fp,vector<vector<double>> &fc, map<int,pair<int,double>> &mp, double delta);
	void swap_check_correspondence_centroidKdtree(vector<vector<double>> &fp,vector<vector<double>> &fc, map<int,pair<int,double>> &mp, double delta);
	public:
		void calculate_correspondence_dtw(vector<vector<double>> &fp, vector<vector<double>> &fc, map<int,pair<int,double>> &mp,double delta);
		void calculate_correspondence_VFHkdtree(vector<vector<double>> &fp, vector<vector<double>> &fc, map<int,pair<int,double>> &mp,double delta);
		void calculate_correspondence_centroidKdtree(vector<vector<double>> &fp, vector<vector<double>> &fc, map<int,pair<int,double>> &mp,double delta);
};