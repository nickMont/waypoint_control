#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Geometry>
#include <string>
#include <iostream>
#include <px4_control/PVA.h>
#include <app_pathplanner_interface/PVATrajectory.h>
#include <app_pathplanner_interface/PVA_Stamped.h>

class waypointControl
{
public:

    /**
    * Constructor.
    */
	waypointControl(ros::NodeHandle &nh);

    /** 
    * Callback called whenever a pose message is published from the state estimator topic 
    */
	void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);

    /**
    * Callback called when a single waypoint is published.
    */
	void wptCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    /**
    * Callback called when a path of waypoints is published. Takes the first element in the list and assigns it to be the next goal position.
    */
	void wptListCallback(const app_pathplanner_interface::PVATrajectory::ConstPtr &msg);

    /**
    * Caps xval between +- satBound
    */
	double saturationF(double &xval, const double satbound);
	void checkArrival(const Eigen::Vector3d &cPose);
	void limitAcceleration(const Eigen::Vector3d &vv, Eigen::Vector3d &uu);

private:

    /**
    * Reads ROS parameters from server into various class variables
    */
    void readROSParameters(); 

	ros::Publisher pvaRef_pub_;
	ros::Subscriber pose_sub_;
	ros::Subscriber waypoint_sub_;
	ros::Subscriber waypointList_sub_, waypointVelList_sub_, waypointAccList_sub_;
	nav_msgs::Odometry::ConstPtr initPose_;
	geometry_msgs::PoseStamped::ConstPtr initWpt_;
	int counter, wptListLen, waypointCounter, numPathsSoFar, stepsToNextWpt, stepCounter;
	double wptTime, poseTime, gpsfps, hitDist, dt_default, t0, dtNextWpt;

	//using Eigen when std::vector could be used instead in case we want to do matrix calcs with this stuff
	Eigen::Vector3d errIntegral, eImax, vmax, amax, arenaCenter, next_wpt, uPID;

    /* PID Parameters */
    Eigen::Vector3d kp, kd, ki;

    /* Previous state vectors */
    Eigen::Vector3d oldPose_, oldVelocity_, oldAcceleration_;

    /* Current state vectors */
    Eigen::Vector3d currentPose_, currentVelocity_;

    /* Future state vectors */
    Eigen::Vector3d nextVelocity_, nextAcceleration_;

	std::string quadPoseTopic, quadWptTopic, publishtopicname, quadWptListTopic,
							quadVelListTopic, quadAccListTopic;
	app_pathplanner_interface::PVATrajectory::ConstPtr global_path_msg;
};

