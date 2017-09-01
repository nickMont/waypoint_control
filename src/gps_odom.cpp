#include <Eigen/Geometry>
#include "gps_odom.hpp"
#include <string>
#include <iostream>

namespace gps_odom
{
gpsOdom::gpsOdom(ros::NodeHandle &nh)
{

  //Get data about node and topic to listen
  std::string quadPoseTopic, quadName;
  quadName = ros::this_node::getName();
  ros::param::get(quadName + "/quadPoseTopic", quadPoseTopic);
  ROS_INFO("Kalman Filter Node started! Listening to ROS topic: %s", quadPoseTopic.c_str());
  ROS_INFO("Waiting for first position measurement...");

  //Get initial pose
  initPose_ = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(quadPoseTopic);
  ROS_INFO("Initial position: %f\t%f\t%f", initPose_->pose.position.x,initPose_->pose.position.y, initPose_->pose.position.z);

  //Get additional parameters for the kalkman filter
  double max_accel;
  nh.param(quadName + "/max_accel", max_accel, 2.0);
  nh.param(quadName + "/publish_tf", publish_tf_, true);
  nh.param<std::string>(quadName + "/child_frame_id", child_frame_id_, "base_link");
  if(publish_tf_ && child_frame_id_.empty())
    throw std::runtime_error("gps_odom: child_frame_id required for publishing tf");

  // There should only be one gps_fps, so we read from nh
  double dt, gps_fps;
  nh.param(quadName + "/gps_fps", gps_fps, 20.0);
  ROS_ASSERT(gps_fps > 0.0);
  dt = 1.0 / gps_fps;

  //verbose parameters
  ROS_INFO("max_accel: %f", max_accel);
  ROS_INFO("publish_tf_: %d", publish_tf_);
  ROS_INFO("child_frame_id: %s", child_frame_id_.c_str());
  ROS_INFO("ROS topic: %s", quadPoseTopic.c_str());
  ROS_INFO("Node name: %s", quadName.c_str());
  ROS_INFO("gps_fps: %f", gps_fps);

  // Initialize KalmanFilter
  KalmanFilter::State_t proc_noise_diag;
  proc_noise_diag(0) = 0.5 * max_accel * dt * dt;
  proc_noise_diag(1) = 0.5 * max_accel * dt * dt;
  proc_noise_diag(2) = 0.5 * max_accel * dt * dt;
  proc_noise_diag(3) = max_accel * dt;
  proc_noise_diag(4) = max_accel * dt;
  proc_noise_diag(5) = max_accel * dt;
  proc_noise_diag = proc_noise_diag.array().square();
  KalmanFilter::Measurement_t meas_noise_diag;
  meas_noise_diag(0) = 1e-2;
  meas_noise_diag(1) = 1e-2;
  meas_noise_diag(2) = 1e-2;
  meas_noise_diag = meas_noise_diag.array().square();
  Eigen::Matrix<double, 6, 1> initStates;
  initStates << initPose_->pose.position.x, initPose_->pose.position.y, initPose_->pose.position.z, 0.0, 0.0, 0.0;
  kf_.initialize(initStates,
                 1.0 * KalmanFilter::ProcessCov_t::Identity(),
                 proc_noise_diag.asDiagonal(), meas_noise_diag.asDiagonal());

  // Initialize publishers and subscriber
  odom_pub_ = nh.advertise<nav_msgs::Odometry>(quadName + "/odom", 10);
  localOdom_pub_ = nh.advertise<nav_msgs::Odometry>(quadName + "/local_odom", 10);
  mocap_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 10);
  gps_sub_ = nh.subscribe(quadPoseTopic, 10, &gpsOdom::gpsCallback,
                            this, ros::TransportHints().tcpNoDelay());
}

void gpsOdom::gpsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  // ROS_INFO("Callback running!");
  static ros::Time t_last_proc = msg->header.stamp;
  static int counter=0;
  static ros::Time time_of_last_fix=msg->header.stamp;
  double lastKnownSpeed=0;
  counter++;
  double dt = (msg->header.stamp - t_last_proc).toSec();
  t_last_proc = msg->header.stamp;
  static Eigen::Vector3d z_last(0.0, 0.0, 0.0);

  // Kalman filter for getting translational velocity from position measurements
  kf_.processUpdate(dt);
  Eigen::Vector3d ECEF;
  ECEF(0) = msg->pose.position.x;
  ECEF(1) = msg->pose.position.y;
  ECEF(2) = msg->pose.position.z;
  const KalmanFilter::Measurement_t meas(msg->pose.position.x, msg->pose.position.y,
                                         msg->pose.position.z);

  static ros::Time t_last_meas = msg->header.stamp; //initialize static variable (basically a scope-limited global)
  double meas_dt = (msg->header.stamp - t_last_meas).toSec();
  t_last_meas = msg->header.stamp; //update static variable after being used
  Eigen::Matrix<double,6,1> xbar=kf_.getState();
  Eigen::Vector3d z_expected;  //propagating distance forwards based on estimated speed
  z_expected(0)=z_last(0)+xbar(3)*dt;
  z_expected(1)=z_last(1)+xbar(4)*dt;
  z_expected(2)=z_last(2)+xbar(5)*dt;
  double dt_last_fix=(msg->header.stamp-time_of_last_fix).toSec();
//  double estDistBySpeed=sqrt(xbar(3)*xbar(3)+xbar(4)*xbar(4)+xbar(5)*xbar(5))*dt_meas;
  double accel_max=2.0;
  double dz_hyp_test = sqrt( pow(z_last(0)-meas(0),2)+pow(z_last(1)-meas(1),2)+pow(z_last(2)-meas(2),2) );
  //maximum expected speed. Normalized by speed rather than pose to handle lost measurement packets
  double hypothesis_test_threshold=(0.5+accel_max*dt_last_fix*dt_last_fix/2+lastKnownSpeed*dt_last_fix)*meas_dt/0.05;
  //ROS_INFO("dt:%f:  dz1: %f:  dz2: %f:  dz3: %f",meas_dt,z_last(0)-meas(0),z_last(1)-meas(1),z_last(2)-meas(2));
  Eigen::Matrix<double, 3, 6> HH;
  HH.setZero();
  HH(0, 0) = 1;
  HH(1, 1) = 1;
  HH(2, 2) = 1;
  Eigen::Vector3d resid = meas - HH*xbar;
  Eigen::Matrix<double,3,3> meas_covmat; //placeholder until I can get the full covariance solution
  meas_covmat.setZero();
  meas_covmat(0,0) = 1;
  meas_covmat(1,1) = 1;
  meas_covmat(2,2) = 1;
  Eigen::Matrix<double,6,6> proc_noise_cov;
  proc_noise_cov = kf_.getProcessNoise();
  //replace residCost with dz_hyp_test when full covariance solution is available
  double residCost=resid.transpose()*(meas_covmat+HH*proc_noise_cov*HH.transpose())*resid; //unused until full covariance is available

  if (dz_hyp_test <= hypothesis_test_threshold)
  {
    kf_.measurementUpdate(meas, meas_dt);
    time_of_last_fix=msg->header.stamp;
    lastKnownSpeed=sqrt(xbar(3)*xbar(3)+xbar(4)*xbar(4)+xbar(5)*xbar(5));
    if (dz_hyp_test>=hypothesis_test_threshold*2/3)
    {
      ROS_INFO("Possible outlier warning at dz=%f", dz_hyp_test);
    }
  }
  else if (counter>1) //bypass first step
  {
    ROS_INFO("Outlier of value %f rejected at count %d", dz_hyp_test,counter);
  }
  //update static variable AFTER using it
  z_last(0)=msg->pose.position.x;
  z_last(1)=msg->pose.position.y;
  z_last(2)=msg->pose.position.z;


  const KalmanFilter::State_t state = kf_.getState();
  const KalmanFilter::ProcessCov_t proc_noise = kf_.getProcessNoise();

  nav_msgs::Odometry odom_msg, localOdom_msg;
  odom_msg.header = msg->header;
  odom_msg.child_frame_id = msg->header.frame_id;
  // odom_msg.child_frame_id = "quadFrame";
  odom_msg.pose.pose.position.x = state(0);
  odom_msg.pose.pose.position.y = state(1);
  odom_msg.pose.pose.position.z = state(2);
  odom_msg.twist.twist.linear.x = state(3);
  odom_msg.twist.twist.linear.y = state(4);
  odom_msg.twist.twist.linear.z = state(5);
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      odom_msg.pose.covariance[6 * i + j] = proc_noise(i, j);
      odom_msg.twist.covariance[6 * i + j] = proc_noise(3 + i, 3 + j);
    }
  }

  odom_msg.pose.pose.orientation = msg->pose.orientation;

  // Single step differentitation for angular velocity
  static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
  Eigen::Matrix3d R(Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                       msg->pose.orientation.y, msg->pose.orientation.z));
  if(dt > 1e-6)
  {
    const Eigen::Matrix3d R_dot = (R - R_prev) / dt;
    const Eigen::Matrix3d w_hat = R_dot * R.transpose();

    odom_msg.twist.twist.angular.x = w_hat(2, 1);
    odom_msg.twist.twist.angular.y = w_hat(0, 2);
    odom_msg.twist.twist.angular.z = w_hat(1, 0);
  }
  R_prev = R;

  odom_pub_.publish(odom_msg);

  //Publish tf  
  if(publish_tf_)
  {
    PublishTransform(odom_msg.pose.pose, odom_msg.header,
                     child_frame_id_);
  }

  //Publish local odometry message
  localOdom_msg = odom_msg;
  localOdom_msg.pose.pose.position.x = localOdom_msg.pose.pose.position.x - initPose_->pose.position.x;
  localOdom_msg.pose.pose.position.y = localOdom_msg.pose.pose.position.y - initPose_->pose.position.y;
  localOdom_msg.pose.pose.position.z = localOdom_msg.pose.pose.position.z - initPose_->pose.position.z;
  localOdom_pub_.publish(localOdom_msg);

  // Publish message for px4 mocap topic

  geometry_msgs::PoseStamped mocap_msg;
  mocap_msg.pose.position.x = msg->pose.position.x - initPose_->pose.position.x;
  mocap_msg.pose.position.y = msg->pose.position.y - initPose_->pose.position.y;
  mocap_msg.pose.position.z = msg->pose.position.z - initPose_->pose.position.z;
  mocap_msg.pose.orientation = msg->pose.orientation;
  mocap_msg.header = msg->header;
  mocap_msg.header.frame_id = "fcu";
  mocap_pub_.publish(mocap_msg);

}

void gpsOdom::PublishTransform(const geometry_msgs::Pose &pose,
                               const std_msgs::Header &header,
                               const std::string &child_frame_id)
{
  // Publish tf
  geometry_msgs::Vector3 translation;
  translation.x = pose.position.x;
  translation.y = pose.position.y;
  translation.z = pose.position.z;

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header = header;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation = translation;
  transform_stamped.transform.rotation = pose.orientation;

  tf_broadcaster_.sendTransform(transform_stamped);
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_odom");
  ros::NodeHandle nh;



  try
  {
    gps_odom::gpsOdom gps_odom(nh);
    ros::spin();
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    return 1;
  }
  return 0;
}
