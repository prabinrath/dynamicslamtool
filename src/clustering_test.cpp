#include "CC/CloudCorrespondence.h"
extern visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster, int id, std::string f_id, std::string ns="bounding_box", float r=0.5, float g=0.5, float b=0.5);

ros::Publisher pub,marker_pub;

boost::shared_ptr<CloudCorrespondence> ca,cb;
boost::shared_ptr<CloudCorrespondenceMethods> mth;

void euclidian_clustering(const sensor_msgs::PointCloud2ConstPtr &input)
{
	sensor_msgs::PointCloud2 output;
  	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  	pcl_conversions::toPCL(*input, *cloud);
  
  	ca = cb;
  	cb.reset(new CloudCorrespondence());
  	pcl::fromPCLPointCloud2(*cloud, *(cb->cloud));
	cb->filterPassThrough(5.0,5.0,5.0);
	cb->computeClusters(0.1,"single_cluster");
	cb->init = true;
	if(ca->init  == true && cb->init == true)
	{
		pcl::toPCLPointCloud2(*(ca->cluster_collection),*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "previous";
		pub.publish(output);
		pcl::toPCLPointCloud2(*(cb->cluster_collection),*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "current";
		pub.publish(output);
  		
	  	map<int,pair<int,double>> mp;
	  	mth->calculate_correspondence_kdtree(ca->vfh_bank,cb->vfh_bank,mp,400); //400:kdtree,120:dtw
	  	float r,g,b;int id = 1;
	  	for(map<int,pair<int,double>>::iterator it=mp.begin();it!=mp.end();it++)
		{
			cout<<"{"<<it->second.first<<"->"<<it->first<<"} Fit Score: "<<it->second.second<<endl;
			r = ((float)(rand()%100))/100; g = ((float)(rand()%100))/100; b = ((float)(rand()%100))/100;
			marker_pub.publish(mark_cluster(ca->clusters[it->first],id,"previous","bounding_box",r,g,b));
			marker_pub.publish(mark_cluster(cb->clusters[it->second.first],id,"current","bounding_box",r,g,b));
			id++;
		}
		cout<<"-----------------------------------------------------\n";
	}
}

int main (int argc, char** argv)
{
  srand((unsigned)time(0));
  ros::init (argc, argv, "test_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, euclidian_clustering);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("bbox", 1);
  ca.reset(new CloudCorrespondence());
  cb.reset(new CloudCorrespondence());
  mth.reset(new CloudCorrespondenceMethods());
  ros::spin();
}