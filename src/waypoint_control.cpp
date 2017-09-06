#include "ros/ros.h"
#include "waypointcontrol.hpp"
#include <Eigen/Geometry>
#include <string>


waypointControl::waypointControl(ros::NodeHandle &nh)
    : counter(0),
      poseTime(0.0),
      wptTime(0.0),
      stepCounter(0.0),
      nextVelocity_(0, 0, 0), 
      nextAcceleration_(0, 0, 0),
      oldPose_(0, 0, 0),
      oldVelocity_(0, 0, 0),
      oldAcceleration_(0, 0, 0),
      numPathsSoFar(0)
{
    this->readROSParameters();

	//set default wpt to origin
    this->next_wpt = this->arenaCenter; // Arena center is takeoff location.
    this->next_wpt(2) = this->next_wpt(2) + 1; // Take off 1 meter off the ground

	this->dt_default= 1.0 / this->gpsfps;

	// Initialize publishers and subscriber
	//advertise on message topic specified in input file.	USE px4_control/PVA_Ref WITH MARCELINO'S CONTROLLER
	pvaRef_pub_ = nh.advertise<px4_control::PVA>(publishtopicname, 10);
	ROS_INFO("Publisher created on topic %s",publishtopicname.c_str());
	pose_sub_ = nh.subscribe(quadPoseTopic, 10, &waypointControl::poseCallback, this, ros::TransportHints().tcpNoDelay());
	waypoint_sub_ = nh.subscribe(quadWptTopic,10,&waypointControl::wptCallback, this, ros::TransportHints().tcpNoDelay());
	waypointList_sub_ = nh.subscribe(quadWptListTopic,10,&waypointControl::wptListCallback, this, ros::TransportHints().tcpNoDelay());
	ROS_INFO("Subscribers successfully created.");

	// Get initial pose
	ROS_INFO("Waiting for first position measurement...");
	initPose_ = ros::topic::waitForMessage<nav_msgs::Odometry>(quadPoseTopic);
	ROS_INFO("Initial position: %f\t%f\t%f", initPose_->pose.pose.position.x, initPose_->pose.pose.position.y, initPose_->pose.pose.position.z);
//	next_wpt(0)=initPose_->pose.pose.position.x;
//	next_wpt(1)=initPose_->pose.pose.position.y;
//	next_wpt(2)=initPose_->pose.pose.position.z+1; //1m above initial pose
}

void waypointControl::readROSParameters() 
{
    // Topic names
	ros::param::get("waypoint_control_node/quadPoseTopic", quadPoseTopic);
	ros::param::get("waypoint_control_node/quadWptTopic", quadWptTopic);
	ros::param::get("waypoint_control_node/quadWptListTopic", quadWptListTopic);
	ros::param::get("waypoint_control_node/publishPVA_Topic", publishtopicname);

	//confirm that parameters were read correctly
	ROS_INFO("Preparing pose subscriber on channel %s",quadPoseTopic.c_str());
	ROS_INFO("Preparing waypoint subscriber on channel %s",quadWptTopic.c_str());
	ROS_INFO("Preparing wptList subscriber on channel %s",quadWptListTopic.c_str());

	// PID Parameters
	ros::param::get("waypoint_control_node/kpX", kp(0));
	ros::param::get("waypoint_control_node/kdX", kd(0));
	ros::param::get("waypoint_control_node/kiX", ki(0));
	ros::param::get("waypoint_control_node/kpY", kp(1));
	ros::param::get("waypoint_control_node/kdY", kd(1));
	ros::param::get("waypoint_control_node/kiY", ki(1));
	ros::param::get("waypoint_control_node/kpZ", kp(2));
	ros::param::get("waypoint_control_node/kdZ", kd(2));
	ros::param::get("waypoint_control_node/kiZ", ki(2));
	ros::param::get("waypoint_control_node/maxInteg_X", eImax(0));
	ros::param::get("waypoint_control_node/maxInteg_Y", eImax(1));
	ros::param::get("waypoint_control_node/maxInteg_Z", eImax(2));

	// Safety parameters. Only currently in use to make intermediate points when initalizing
	ros::param::get("waypoint_control_node/vx_max", vmax(0));
	ros::param::get("waypoint_control_node/vy_max", vmax(1));
	ros::param::get("waypoint_control_node/vz_max", vmax(2));
	ros::param::get("waypoint_control_node/gps_fps", gpsfps);
	ros::param::get("waypoint_control_node/waypointHitDist", hitDist);
	ros::param::get("waypoint_control_node/ax_max", amax(0));
	ros::param::get("waypoint_control_node/ay_max", amax(1));
	ros::param::get("waypoint_control_node/az_max", amax(2));
	ros::param::get("waypoint_control_node/xCenter", arenaCenter(0));
	ros::param::get("waypoint_control_node/yCenter", arenaCenter(1));
	ros::param::get("waypoint_control_node/zCenter", arenaCenter(2));

}

void waypointControl::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ //Sends PVA_Ref based on current PVA when received

	//Send new PVA when new KFPose received
	//static ros::Time t_last_proc = msg->header.stamp;
	//double dt = (msg->header.stamp - t_last_proc).toSec();
	//t_last_proc = msg->header.stamp;
	
	//PID3 structure can also be used here
	currentPose_(0)=msg->pose.pose.position.x;
	currentPose_(1)=msg->pose.pose.position.y;
	currentPose_(2)=msg->pose.pose.position.z;
	currentVelocity_(0)=msg->twist.twist.linear.x;
	currentVelocity_(1)=msg->twist.twist.linear.y;
	currentVelocity_(2)=msg->twist.twist.linear.z;
//	currentPose_(0)=0; currentPose_(1)=0; currentPose_(2)=0;
//	currentVelocity_(0)=0; currentVelocity_(1)=0; currentVelocity_(2)=0;

	checkArrival(currentPose_);

//	errIntegral(0)=dt_default*(next_wpt(0)-currentPose_(0));
//	errIntegral(1)=dt_default*(next_wpt(1)-currentPose_(1));
//	errIntegral(2)=dt_default*(next_wpt(2)-currentPose_(2));

//	//Integrator saturation
//	saturationF(errIntegral(0),eImax(0));
//	saturationF(errIntegral(1),eImax(1));
//	saturationF(errIntegral(2),eImax(2));

//	//create PID command
//	uPID(0)=kp(0)*(next_wpt(0)-currentPose_(0)) + kd(0)*(0-currentVelocity_(0)) + ki(0)*errIntegral(0);
//	uPID(1)=kp(1)*(next_wpt(1)-currentPose_(1)) + kd(1)*(0-currentVelocity_(1)) + ki(1)*errIntegral(1);
//	uPID(2)=kp(2)*(next_wpt(2)-currentPose_(2)) + kd(2)*(0-currentVelocity_(2)) + ki(2)*errIntegral(2);

//	limitAcceleration(currentVelocity_,uPID);

	//uses the PVA message from px4_control package
	px4_control::PVA PVA_Ref_msg;
	PVA_Ref_msg.yaw=0.0;	//can try nonzero if optimal

//	//Move to next point with PID
//	PVA_Ref_msg.Pos.x=currentPose_(0)+dt_default*currentVelocity_(0)+dt_default*dt_default*0.5*uPID(0);
//	PVA_Ref_msg.Pos.y=currentPose_(1)+dt_default*currentVelocity_(1)+dt_default*dt_default*0.5*uPID(1);
//	PVA_Ref_msg.Pos.z=currentPose_(2)+dt_default*currentVelocity_(2)+dt_default*dt_default*0.5*uPID(2);
//	PVA_Ref_msg.Vel.x=currentVelocity_(0)+dt_default*uPID(0);
//	PVA_Ref_msg.Vel.y=currentVelocity_(1)+dt_default*uPID(1);
//	PVA_Ref_msg.Vel.z=currentVelocity_(2)+dt_default*uPID(2);
//	PVA_Ref_msg.Acc.x=uPID(0);
//	PVA_Ref_msg.Acc.y=uPID(1);
//	PVA_Ref_msg.Acc.z=uPID(2);

//	//Move to next wpt in one step
//	PVA_Ref_msg.Pos.x=next_wpt(0);
//	PVA_Ref_msg.Pos.y=next_wpt(1);
//	PVA_Ref_msg.Pos.z=next_wpt(2);
//	PVA_Ref_msg.Vel.x=0;
//	PVA_Ref_msg.Vel.y=0;
//	PVA_Ref_msg.Vel.z=0;
//	PVA_Ref_msg.Acc.x=0;
//	PVA_Ref_msg.Acc.y=0;
//	PVA_Ref_msg.Acc.z=0;

	//Move to next wpt in multiple substeps
	//Check to see if the window needs to increase in size
	double dt_x = (next_wpt(0)-currentPose_(0))/vmax(0);
	double dt_y = (next_wpt(1)-currentPose_(1))/vmax(1);
	double dt_z = (next_wpt(2)-currentPose_(2))/vmax(2);
	double estStepsMaxVel = ceil(std::max(dt_x,std::max(dt_y,dt_z)) / dt_default);
	if(estStepsMaxVel>stepsToNextWpt-stepCounter) stepsToNextWpt++;
	//get substep size
	stepCounter++;
	double substep=1-stepCounter*(1.0/stepsToNextWpt);

	//fill message fields
	//The proper way to do this is with discrete integration but handling it with substeps is close enough
	PVA_Ref_msg.Pos.x=next_wpt(0)-substep*(next_wpt(0)-oldPose_(0));
	PVA_Ref_msg.Pos.y=next_wpt(1)-substep*(next_wpt(1)-oldPose_(1));
	PVA_Ref_msg.Pos.z=next_wpt(2)-substep*(next_wpt(2)-oldPose_(2));
	PVA_Ref_msg.Vel.x=nextVelocity_(0)-substep*(nextVelocity_(0)-oldVelocity_(0));
	PVA_Ref_msg.Vel.y=nextVelocity_(1)-substep*(nextVelocity_(1)-oldVelocity_(1));
	PVA_Ref_msg.Vel.z=nextVelocity_(2)-substep*(nextVelocity_(2)-oldVelocity_(2));
	PVA_Ref_msg.Acc.x=nextAcceleration_(0)-substep*(nextAcceleration_(0)-oldAcceleration_(0));
	PVA_Ref_msg.Acc.y=nextAcceleration_(1)-substep*(nextAcceleration_(1)-oldAcceleration_(1));
	PVA_Ref_msg.Acc.z=nextAcceleration_(2)-substep*(nextAcceleration_(2)-oldAcceleration_(2));

	pvaRef_pub_.publish(PVA_Ref_msg);
}


void waypointControl::wptListCallback(const app_pathplanner_interface::PVATrajectory::ConstPtr &msg)
{
	//int nn= ;//length of pose list
	numPathsSoFar++;
	if(msg) {
		wptListLen=msg->pva.size();
		global_path_msg = msg;

		//initialize waypoint counter
		waypointCounter=0;
		counter++; //total number of waypoint list/paths received

		//set old states to 0
		oldPose_=currentPose_;
		oldVelocity_=currentVelocity_;
		oldAcceleration_(0)=0;
		oldAcceleration_(1)=0;
		oldAcceleration_(2)=0;

		//reset PID params
		errIntegral(0)=0.0;
		errIntegral(1)=0.0;
		errIntegral(2)=0.0;

		//get next waypoint PVA from list
		next_wpt(0) = global_path_msg->pva[0].pos.position.x;
		next_wpt(1) = global_path_msg->pva[0].pos.position.y;
		next_wpt(2) = global_path_msg->pva[0].pos.position.z;
		nextVelocity_(0) = global_path_msg->pva[0].vel.linear.x;
		nextVelocity_(1) = global_path_msg->pva[0].vel.linear.y;
		nextVelocity_(2) = global_path_msg->pva[0].vel.linear.z;
		nextAcceleration_(0) = global_path_msg->pva[0].acc.linear.x;
		nextAcceleration_(1) = global_path_msg->pva[0].acc.linear.y;
		nextAcceleration_(2) = global_path_msg->pva[0].acc.linear.z;

		//estimated steps to next wpt
		double dt_x = (next_wpt(0)-currentPose_(0))/vmax(0);
		double dt_y = (next_wpt(1)-currentPose_(1))/vmax(1);
		double dt_z = (next_wpt(2)-currentPose_(2))/vmax(2);
		stepsToNextWpt = ceil(std::max(dt_x,std::max(dt_y,dt_z)) / dt_default);
		ROS_INFO("Moving to waypoint %f %f %f",next_wpt(0),next_wpt(1),next_wpt(2));
	}
    else
	{
		next_wpt=currentPose_;
	}
}


//if a single wpt is received
void waypointControl::wptCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	double tempTime=(msg->header.stamp).toSec();
	if(tempTime>wptTime)	//if an update arrives late, ignore it
	{
		wptTime=tempTime;
		//read in next wpt and offset by distance between origin and center
		next_wpt(0)=msg->pose.position.x+arenaCenter(0);
		next_wpt(1)=msg->pose.position.y+arenaCenter(1);
		next_wpt(2)=msg->pose.position.z+arenaCenter(2);
		nextVelocity_.setZero();
		nextAcceleration_.setZero();
		oldPose_=currentPose_;
		oldVelocity_=currentVelocity_;
		oldAcceleration_.setZero();
		errIntegral(0)=0.0;
		errIntegral(1)=0.0;
		errIntegral(2)=0.0;
	}
}


double waypointControl::saturationF(double &xval, const double satbound)
{
	if(xval>satbound){		
		xval=satbound;
	}else if(xval<-satbound){
		xval=-satbound;
	}
}


void waypointControl::limitAcceleration(const Eigen::Vector3d &vv, Eigen::Vector3d &uu)
{
	Eigen::Vector3d velWithAccel;
	double aCheck;
	velWithAccel=vv+dt_default*uu;

	for(int i=0;i<3;i++)
	{
		if(velWithAccel(i)>vmax(i)) //handles positive rel speed
		{
			aCheck=(vmax(i)-velWithAccel(i))/dt_default;
			if(abs(aCheck)>amax(i))
			{
				uu(i)=abs(aCheck)/aCheck*amax(i); //sign(a) * a_max
			}else
			{
				uu(i)=aCheck; //do not use limit if unnecessary
			}
		}else if(velWithAccel(i)<-vmax(i)) //handles negative rel speed
		{
			aCheck=(-vmax(i)-velWithAccel(i))/dt_default;
			if(abs(aCheck)>amax(i))
			{
				uu(i)=abs(aCheck)/aCheck*amax(i); //sign(a) * a_max
			}else
			{
				uu(i)=aCheck; //don't unnecessarily limit acceleration
			}
		} //else //all good
	}
}


void waypointControl::checkArrival(const Eigen::Vector3d &cPose)
{
	Eigen::Vector3d thisWpt;
	if(!global_path_msg) return;
	thisWpt(0)=global_path_msg->pva[waypointCounter].pos.position.x+arenaCenter(0);
	thisWpt(1)=global_path_msg->pva[waypointCounter].pos.position.y+arenaCenter(1);
	thisWpt(2)=global_path_msg->pva[waypointCounter].pos.position.z+arenaCenter(2);

	double thisDist=sqrt(pow(thisWpt(0)-cPose(0),2) + pow(thisWpt(1)-cPose(1),2) + pow(thisWpt(2)-cPose(2),2));
//	ROS_INFO("distcurr = %f",thisDist-hitDist);
	if(thisDist<hitDist)
	{
//		t0=time0;
		if(waypointCounter<wptListLen-1)
		{
			waypointCounter++;
			//update new and old wpt locations
			oldPose_=next_wpt;
			next_wpt(0)=global_path_msg->pva[waypointCounter].pos.position.x+arenaCenter(0);
			next_wpt(1)=global_path_msg->pva[waypointCounter].pos.position.y+arenaCenter(1);
			next_wpt(2)=global_path_msg->pva[waypointCounter].pos.position.z+arenaCenter(2);

			oldVelocity_=nextVelocity_;
			nextVelocity_(0)=global_path_msg->pva[waypointCounter].vel.linear.x;
			nextVelocity_(1)=global_path_msg->pva[waypointCounter].vel.linear.y;
			nextVelocity_(2)=global_path_msg->pva[waypointCounter].vel.linear.z;

			oldAcceleration_=nextAcceleration_;
			nextAcceleration_(0)=global_path_msg->pva[waypointCounter].acc.linear.x;
			nextAcceleration_(1)=global_path_msg->pva[waypointCounter].acc.linear.y;
			nextAcceleration_(2)=global_path_msg->pva[waypointCounter].acc.linear.z;

			//if you have the next time, find segment chunks that correspond to it. Otherwise, guesstimate.
			if(global_path_msg->pva[waypointCounter].header.stamp.toSec() > 1e-4 )
			{
				dtNextWpt=(global_path_msg->pva[waypointCounter].header.stamp).toSec() -
									 (global_path_msg->pva[waypointCounter-1].header.stamp).toSec();			
			}
            else
			{
				double dt_x = (next_wpt(0)-cPose(0))/vmax(0);
				double dt_y = (next_wpt(1)-cPose(1))/vmax(1);
				double dt_z = (next_wpt(2)-cPose(2))/vmax(2);
				dtNextWpt=std::max(std::max(dt_x,dt_y),dt_z);
			}
			stepsToNextWpt=ceil(dtNextWpt/dt_default);
			stepCounter=0;
			errIntegral(0)=0.0;
			errIntegral(1)=0.0;
			errIntegral(2)=0.0;
			ROS_INFO("Waypoint reached, moving to waypoint %f\t%f\t%f",next_wpt(0),next_wpt(1),next_wpt(2));
		}
        else
		{
			ROS_INFO("Final waypoint reached, holding station.");

			//Set old VA to 0 to hold station
			nextVelocity_.setZero();
			nextAcceleration_.setZero();
			oldVelocity_.setZero();
			oldAcceleration_.setZero();
		}
	}
}


