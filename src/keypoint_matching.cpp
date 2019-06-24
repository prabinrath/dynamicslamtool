#include "CC/CloudCorrespondence.h"

ros::Publisher pub;

boost::shared_ptr<CloudCorrespondence> ca,cb;

void key_point_correspondence(const sensor_msgs::PointCloud2ConstPtr &input)
{
	sensor_msgs::PointCloud2 output;
  	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  	pcl_conversions::toPCL(*input, *cloud);
  
  	ca = cb;
  	cb.reset(new CloudCorrespondence());
  	pcl::fromPCLPointCloud2(*cloud, *(cb->cloud)); 	

  	pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> norm_est;
  	norm_est.setKSearch (20);
	norm_est.setInputCloud (cb->cloud);
	norm_est.compute (*cb->nor);

	pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZI> sift;
    sift.setRadiusSearch(0.1);
    sift.setScales(0.01, 3, 4);
    sift.setMinimumContrast(0.001);
    sift.setInputCloud(cb->cloud);
	sift.compute (*cb->cluster_collection);
	//cout<<cb->cloud->points.size()<<" "<<cb->cluster_collection->points.size()<<endl;

	cb->init = true;

	if(ca->init  == true && cb->init == true)
	{
    	/*pcl::SHOTEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::SHOT352> descr_est;
  		descr_est.setRadiusSearch (0.05);
  		descr_est.setInputCloud (ca->cluster_collection);
  		descr_est.setInputNormals (ca->nor);
  		descr_est.setSearchSurface (ca->cloud);
  		descr_est.compute (*ca->ld);
  		descr_est.setInputCloud (cb->cluster_collection);
  		descr_est.setInputNormals (cb->nor);
  		descr_est.setSearchSurface (cb->cloud);
  		descr_est.compute (*cb->ld);*/

  		/*pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
	  	fpfh.setInputCloud (ca->cluster_collection);
	  	fpfh.setInputNormals (ca->nor);
	  	fpfh.setSearchSurface (ca->cloud);
	  	fpfh.setRadiusSearch (0.05);
	  	fpfh.compute (*ca->ld);
	  	fpfh.setInputCloud (cb->cluster_collection);
	  	fpfh.setInputNormals (cb->nor);
	  	fpfh.setSearchSurface (cb->cloud);
	  	fpfh.setRadiusSearch (0.05);
	  	fpfh.compute (*cb->ld);*/  	

	  	//cout<<ca->ld->points.size()<<" "<<cb->ld->points.size()<<endl;

  		pcl::CorrespondencesPtr corrs(new pcl::Correspondences ());
  		pcl::registration::CorrespondenceEstimation<pcl::PointXYZI, pcl::PointXYZI> corr_est;
  		corr_est.setInputSource (ca->cluster_collection);
  		corr_est.setInputTarget (cb->cluster_collection);
		corr_est.determineReciprocalCorrespondences (*corrs);

		pcl::PointCloud<pcl::PointXYZI> corrs_ca,corrs_cb;
		for(int i=0;i<corrs->size();i++)
		{
			//pcl::Correspondence curr = (*corrs)[i];
			corrs_ca.points.push_back(ca->cluster_collection->points[(*corrs)[i].index_query]);
			corrs_cb.points.push_back(cb->cluster_collection->points[(*corrs)[i].index_match]);
		}
  		
		cout<<corrs_ca.points.size()<<" "<<corrs_ca.points.size()<<endl;

		pcl::toPCLPointCloud2(corrs_ca,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "previous";
		pub.publish(output);

		pcl::toPCLPointCloud2(corrs_cb,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "current";
		pub.publish(output);
	}
}

int main (int argc, char** argv)
{
  srand((unsigned)time(0));
  ros::init (argc, argv, "test_keypoint_matching");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 10, key_point_correspondence);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 10);
  ca.reset(new CloudCorrespondence());
  cb.reset(new CloudCorrespondence());

  ros::spin();
}