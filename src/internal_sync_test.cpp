#include "MOR/MovingObjectRemoval.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "test_moving_object");
  ros::NodeHandle nh;

  MovingObjectRemoval mor(nh,"/home/ar2s/catkin_ws/src/Research_stages/config/MOR_config.txt",3,3);

  ros::spin();
}