#ifndef GPS_ODOM_NODE_HPP_
#define GPS_ODOM_NODE_HPP_

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "filter.h"

namespace gps_odom
{
class gpsOdom
{
 public:
  gpsOdom(ros::NodeHandle &nh);

  void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

 private:
  void PublishTransform(const geometry_msgs::Pose &pose,
                        const std_msgs::Header &header,
                        const std::string &child_frame_id);

  gps_odom::KalmanFilter kf_;
  ros::Publisher odom_pub_;
  ros::Publisher localOdom_pub_;
  ros::Publisher mocap_pub_;
  std::string child_frame_id_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  bool publish_tf_;
  ros::Subscriber gps_sub_;
  geometry_msgs::PoseStamped::ConstPtr initPose_;
};

} // gps_odom

#endif // GPS_ODOM_NODE_HPP_
