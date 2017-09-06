#include "waypointcontrol.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_Control");
  ros::NodeHandle nh;

  try
  {
    waypoint_control::waypointControl waypoint_control(nh);
    ros::spin();
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    return 1;
  }
  return 0;
}

