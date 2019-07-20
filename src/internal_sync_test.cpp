#include "MOR/MovingObjectRemoval.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "dynamic_slam_tool");
  ros::NodeHandle nh;

  MovingObjectRemoval mor(nh,"under_dev",4,3);

  ros::spin();
}