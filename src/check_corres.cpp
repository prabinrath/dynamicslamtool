#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <utility>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <flann/flann.h>
#include <dtw/dtw.h>
using namespace std;

void print(vector<double> vec)
{
	if(vec.size()==0)
	{
		cout<<"Empty\n";
	}
	cout<<vec.size();
	for(int i=0;i<vec.size();i++)
	{
		cout<<vec[i]<<" ";
	}cout<<endl;
}

void calculate_correspondence_dtw(vector<vector<double>> fp,vector<vector<double>> fc)
{
	cout<<fp.size()<<" "<<fc.size()<<endl;
	map<int,pair<int,double>> mp;
	for(int i=0;i<fp.size();i++)
	{
		LB_ImprovedEarly filter(fp[i], fp[i].size() / 2);
		double bestfit = filter.getLowestCost();
		int ind = 0;
		for(int j=0;j<fc.size();j++)
		{
			double curr = filter.test(fc[j]);
			if(curr<bestfit)
			{
				bestfit = curr;
				ind = j;
			}
		}

		if(mp.find(ind)==mp.end())
		{
			mp[ind]={i,bestfit};
		}
		else
		{
			if(mp[ind].second>bestfit)
			{
				mp[ind]={i,bestfit};
			}
		}
	}

	for(map<int,pair<int,double>>::iterator it=mp.begin();it!=mp.end();it++)
	{
		cout<<"{"<<it->second.first<<"->"<<it->first<<"} Fit Score: "<<it->second.second<<endl;
	}
	cout<<"-----------------------------------------------------\n";
}

void calculate_correspondence_kdtree(vector<vector<double>> fp,vector<vector<double>> fc)
{
	cout<<fp.size()<<" "<<fc.size()<<endl;
	pcl::PointCloud<pcl::VFHSignature308>::Ptr cldp(new pcl::PointCloud<pcl::VFHSignature308>());
	for(int i=0;i<fp.size();i++)
	{
		pcl::VFHSignature308 pt;
		for(int j=0;j<fp[i].size();j++)
		{
			pt.histogram[j] = fp[i][j];
		}
		cldp->points.push_back(pt);
	}
	
	pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::VFHSignature308>);
	kdtree->setInputCloud(cldp);
	map<int,pair<int,float>> mp;

	for(int i=0;i<fc.size();i++)
	{
		pcl::VFHSignature308 pt;
		for(int j=0;j<fc[i].size();j++)
		{
			pt.histogram[j] = fc[i][j];
		}

		int K = 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);

		if(kdtree->nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  		{
  				if(mp.find(pointIdxNKNSearch[0]) == mp.end())
  				{
  					mp[pointIdxNKNSearch[0]] = {i,pointNKNSquaredDistance[0]};
  				}
  				else
  				{
  					if(mp[pointIdxNKNSearch[0]].second>pointNKNSquaredDistance[0])
  					{
  						mp[pointIdxNKNSearch[0]] = {i,pointNKNSquaredDistance[0]};
  					}
  				}
  		}
	}

	for(map<int,pair<int,float>>::iterator it=mp.begin();it!=mp.end();it++)
	{
		cout<<"{"<<it->first<<"->"<<it->second.first<<"} Fit Score: "<<it->second.second<<endl;
	}
	cout<<"-----------------------------------------------------\n";
}

int main(int argc,char* args[])
{
	ifstream vfhfile(args[1]);
	if(!vfhfile.is_open())
	{
		cout<<"Couldn't open file\n";
		return 0;
	}
	
	string line;
	int frame=0;
	vector<vector<double>> fp,fc;
	while(getline(vfhfile, line) && vfhfile.is_open())
	{
		if(line[0]=='#')
		{
			frame++;
			if(fp.size()!=0 && fc.size()!=0)
			{
				if(frame%2==0)
				{
					calculate_correspondence_kdtree(fp,fc);
					fp.clear();
				}
				else
				{
					calculate_correspondence_kdtree(fc,fp);
					fc.clear();
				}
			}
			continue;
		}
		
		string temp="";
		vector<double> vfh;
		for(int i=0;i<line.size();i++)
		{
			if(line[i]!=',')
			{
				temp.push_back(line[i]);
			}
			else
			{
				vfh.push_back(atof(temp.c_str()));
				temp="";
			}
		}

		if(frame%2==0)
		{
			fp.push_back(vfh);
		}
		else
		{
			fc.push_back(vfh);
		}
	}
	vfhfile.close();
}
