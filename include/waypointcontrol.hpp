#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Geometry>
#include <string>
#include <iostream>
#include <px4_control/PVA.h>
#include <px4_control/updatePx4param.h>
//#include <px4_control/srv/updatePx4param.srv>
#include <mg_msgs/PVATrajectory.h>
#include <mg_msgs/PVA_Stamped.h>

class waypointControl
{
public:

    /**
    * Constructor.
    */
	waypointControl(ros::NodeHandle &nh);

    /**
    * Callback that resets control param when joy mode changes
    */
    void joyCallback(const sensor_msgs::Joy &msg);
    /** 
    * Callback called whenever a pose message is published from the state estimator topic 
    */
	void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);

    /**
    * Callback called when a single waypoint is published.
    */
	void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    /**
    * Callback called when a path of waypoints is published. Takes the first element in the list and assigns it to be the next goal position.
    */
	void waypointListCallback(const mg_msgs::PVATrajectory::ConstPtr &msg);

    /**
    * Caps xval between +- satBound
    */
	double saturationF(double &xval, const double satbound);
	void checkArrival(const Eigen::Vector3d &cPose);
    void updateArrivalTiming(const Eigen::Vector3d &cPose);
	void limitAcceleration(const Eigen::Vector3d &vv, Eigen::Vector3d &uu);
    Eigen::Vector3d vectorSaturationF(Eigen::Vector3d &vec1, const Eigen::Vector3d &vecSatbound);

private:

    /**
    * Reads ROS parameters from server into various class variables
    */
    void readROSParameters(); 

	ros::Publisher pvaRef_pub_;
	ros::Subscriber waypoint_sub_, joy_sub_, pose_sub_, waypointList_sub_, waypointVelList_sub_, waypointAccList_sub_;
    ros::ServiceClient controlParamUpdate; 

	nav_msgs::Odometry::ConstPtr initPose_;
	geometry_msgs::PoseStamped::ConstPtr initWaypoint_;
	int counter, waypointListLen, waypointCounter, numPathsSoFar, stepsToNextWaypoint, stepCounter,
        arrivalModeFlag;
	double waypointTime, poseTime, gpsfps, hitDist, dt_default, t0, dtNextWaypoint,
            vmax_for_timing, vmax_real, quadMass, nextYaw_, PI, max_accel;
    
	//using Eigen when std::vector could be used instead in case we want to do matrix calcs with this stuff
	Eigen::Vector3d errIntegral, vmax, arenaCenter, nextWaypoint_, uPID;

    /* PID Parameters */
    Eigen::Vector3d kp, kd, ki, eImax, kp_pos, kd_pos, ki_pos, eImax_pos, kf_ff, kp_hov, kd_hov, ki_hov, eImax_hov;

    /* Previous state vectors */
    Eigen::Vector3d oldPose_, oldVelocity_, oldAcceleration_;

    /* Current state vectors */
    Eigen::Vector3d currentPose_, currentVelocity_;

    /* Future state vectors */
    Eigen::Vector3d nextVelocity_, nextAcceleration_;

	std::string quadPoseTopic, quadWaypointTopic, publishtopicname, quadWaypointListTopic,
							quadVelListTopic, quadAccListTopic, joyTopic, default_mode;
	mg_msgs::PVATrajectory::ConstPtr global_path_msg;
};

