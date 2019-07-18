#include "MOR/MovingObjectRemoval.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "test_moving_object");
  ros::NodeHandle nh;

  MovingObjectRemoval mor(nh,"under_dev",3,3);

  ros::spin();
}