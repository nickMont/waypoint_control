#ifndef WAYPOINT_NODE_HPP_
#define WAYPOINT_NODE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Geometry>
#include <string>
#include <iostream>
#include <px4_control/PVA.h>

namespace waypoint_control
{
class waypointControl
{
 public:
  waypointControl(ros::NodeHandle &nh);

  void poseCallback_error(const nav_msgs::Odometry::ConstPtr &msg);
  void wptCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void wptListCallback(const nav_msgs::Path::ConstPtr &msg);
  void wptVelListCallback(const nav_msgs::Path::ConstPtr &msg);
  void wptAccListCallback(const nav_msgs::Path::ConstPtr &msg);
  double saturationF(double& xval, const double satbound);
  void checkArrival(const Eigen::Vector3d &cPose);
  void limitAcceleration(const Eigen::Vector3d &vv, Eigen::Vector3d &uu);
  void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);

  ros::Publisher pvaRef_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber waypointList_sub_;
  nav_msgs::Odometry::ConstPtr initPose_;
  geometry_msgs::PoseStamped::ConstPtr initWpt_;
  int counter, wptListLen, waypointCounter;
  double wptTime, poseTime, gpsfps, hitDist, dt_default, t0;
  //using Eigen when std::vector could be used instead in case we want to do matrix calcs with this stuff
  Eigen::Vector3d errIntegral, kp, kd, ki, eImax, vmax, amax, arenaCenter, next_wpt, next_vel, next_acc, uPID, poseCurr, velCurr;
  std::string quadPoseTopic, quadName, quadWptTopic, publishtopicname, quadWptListTopic;
  nav_msgs::Path::ConstPtr global_msg;
};

} // gps_odom

#endif // WAYPOINT_NODE_HPP_
