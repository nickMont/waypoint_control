#include "waypointcontrol.hpp"

waypointControl::waypointControl(ros::NodeHandle &nh)
    : counter(0),
      poseTime(0.0),
      waypointTime(0.0),
      stepCounter(0.0),
      nextVelocity_(0, 0, 0), 
      nextAcceleration_(0, 0, 0),
      oldPose_(0, 0, 0),
      oldVelocity_(0, 0, 0),
      oldAcceleration_(0, 0, 0),
      numPathsSoFar(0)
{
    this->readROSParameters();

	// Arena center is takeoff location. +1 meter to ensure takeoff. 
    this->nextWaypoint_ = this->arenaCenter + Eigen::Vector3d(0, 0, 1);

	this->dt_default= 1.0 / this->gpsfps;

	// Initialize publishers and subscriber
	//advertise on message topic specified in input file.	USE px4_control/PVA_Ref WITH MARCELINO'S CONTROLLER
	pvaRef_pub_ = nh.advertise<px4_control::PVA>(publishtopicname, 10);
	ROS_INFO("Publisher created on topic %s",publishtopicname.c_str());
	pose_sub_ = nh.subscribe(quadPoseTopic, 10, &waypointControl::poseCallback, this, ros::TransportHints().tcpNoDelay());
	waypoint_sub_ = nh.subscribe(quadWaypointTopic,10,&waypointControl::waypointCallback, this, ros::TransportHints().tcpNoDelay());
	waypointList_sub_ = nh.subscribe(quadWaypointListTopic,10,&waypointControl::waypointListCallback, this, ros::TransportHints().tcpNoDelay());
	ROS_INFO("Subscribers successfully created.");

//	//get initial pose
//	ROS_INFO("Waiting for first position measurement...");
//	initPose_ = ros::topic::waitForMessage<nav_msgs::Odometry>(quadPoseTopic);
//	ROS_INFO("Initial position: %f\t%f\t%f", initPose_->pose.pose.position.x, initPose_->pose.pose.position.y, initPose_->pose.pose.position.z);
//	next_wpt(0)=initPose_->pose.pose.position.x;
//	next_wpt(1)=initPose_->pose.pose.position.y;
//	next_wpt(2)=initPose_->pose.pose.position.z+1; //1m above initial pose
}

void waypointControl::readROSParameters() 
{
    // Topic names
	ros::param::get("waypoint_control_node/quadPoseTopic", quadPoseTopic);
	ros::param::get("waypoint_control_node/quadWaypointTopic", quadWaypointTopic);
	ros::param::get("waypoint_control_node/quadWaypointListTopic", quadWaypointListTopic);
	ros::param::get("waypoint_control_node/publishPVA_Topic", publishtopicname);

	//confirm that parameters were read correctly
	ROS_INFO("Preparing pose subscriber on channel %s",quadPoseTopic.c_str());
	ROS_INFO("Preparing waypoint subscriber on channel %s",quadWaypointTopic.c_str());
	ROS_INFO("Preparing waypointList subscriber on channel %s",quadWaypointListTopic.c_str());

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

	ros::param::get("waypoint_control_node/vmax_for_timing",vmax_for_timing);
	ros::param::get("waypoint_control_node/vmax_real",vmax_real);

}

void waypointControl::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ //Sends PVA_Ref based on current PVA when received
	//Send new PVA when new KFPose received
	//static ros::Time t_last_proc = msg->header.stamp;
	//double dt = (msg->header.stamp - t_last_proc).toSec();
	//t_last_proc = msg->header.stamp;
	
	//PID3 structure can also be used here
    this->currentPose_ = Eigen::Vector3d(
	    msg->pose.pose.position.x,
	    msg->pose.pose.position.y,
	    msg->pose.pose.position.z
    );

    this->currentVelocity_ = Eigen::Vector3d(
	    msg->twist.twist.linear.x,
	    msg->twist.twist.linear.y,
	    msg->twist.twist.linear.z
    );


    //Prepare for multiple cases
   	double substep;
	//uses the PVA message from px4_control package
	px4_control::PVA PVA_Ref_msg;
	PVA_Ref_msg.yaw = 0.0;	//can try nonzero if optimal

	//if there is a path, follow it.  If not, hover near arena center
    if(global_path_msg)
    {
		checkArrival(currentPose_);
//		limitAcceleration(currentVelocity_,uPID);

		//Move to next waypoint in multiple substeps
		//Check to see if the window needs to increase in size
 	    Eigen::Vector3d dt = (nextWaypoint_ - currentPose_)/vmax_real;
		double estStepsMaxVel = std::ceil(dt.lpNorm<Eigen::Infinity>() / dt_default);

		if(estStepsMaxVel > stepsToNextWaypoint - stepCounter) //+1 to account for noise 
 	    {
 	       stepCounter--;
	    }

		//get substep size
		stepCounter++;

		//saturate substep size
		substep=1-stepCounter*(1.0/stepsToNextWaypoint);
		double subtemp=substep;
		if(substep<0)
		{
			substep=0;
		}else if(substep>1)
		{
			substep=1;
		}
	}else  //if no message has been received
	{
		//set next wpt to be stationary at arena center
		nextWaypoint_=arenaCenter;
		nextWaypoint_(2)=arenaCenter(2)+1;
		nextVelocity_.setZero();
		nextAcceleration_.setZero();
		oldPose_=currentPose_;
//		oldVelocity_=currentVelocity_;
		oldVelocity_.setZero();
		oldAcceleration_.setZero();

// 	    Eigen::Vector3d dt = (nextWaypoint_ - currentPose_).cwiseQuotient(vmax);
 	    Eigen::Vector3d dt = (nextWaypoint_ - currentPose_)/vmax_for_timing;
		double estStepsMaxVel = std::ceil(dt.lpNorm<Eigen::Infinity>() / dt_default);
		substep=1-1/estStepsMaxVel;
		ROS_INFO("Substep: %f",substep);
	}

	//ROS_INFO("subcount %f  zdist %f",substep,nextWaypoint_(2)-substep*(nextWaypoint_(2)-oldPose_(2)));
	
	//fill message fields
	//The proper way to do this is with discrete integration but handling it with substeps is close enough
    Eigen::Vector3d tmp;

    tmp = nextWaypoint_ - substep * (nextWaypoint_ - oldPose_);
	PVA_Ref_msg.Pos.x = tmp(0);
	PVA_Ref_msg.Pos.y = tmp(1);
	PVA_Ref_msg.Pos.z = tmp(2);
	ROS_INFO("x: %f  y: %f  z: %f",tmp(0),tmp(1),tmp(2));

    tmp = nextVelocity_ - substep * (nextVelocity_ - oldVelocity_);
	PVA_Ref_msg.Vel.x = tmp(0);
	PVA_Ref_msg.Vel.y = tmp(1);
	PVA_Ref_msg.Vel.z = tmp(2);

    tmp = nextAcceleration_ - substep * (nextAcceleration_ - oldAcceleration_);
	PVA_Ref_msg.Acc.x = tmp(0);
	PVA_Ref_msg.Acc.y = tmp(1);
	PVA_Ref_msg.Acc.z = tmp(2);

	pvaRef_pub_.publish(PVA_Ref_msg);
	/*
	//static double zprev=
	ROS_INFO("dz=%f");*/
}


void waypointControl::waypointListCallback(const app_pathplanner_interface::PVATrajectory::ConstPtr &msg)
{
	//int nn= ;//length of pose list
	numPathsSoFar++;
	if(msg) {
		waypointListLen=msg->pva.size();
		global_path_msg = msg;

		//initialize waypoint counter
		waypointCounter=0;
		counter++; //total number of waypoint list/paths received

		//set old states to 0
		oldPose_ = currentPose_;
		oldVelocity_ = currentVelocity_;
        oldAcceleration_.setZero();

		//reset PID params
        errIntegral.setZero();

		//get next waypoint PVA from list
        nextWaypoint_ = Eigen::Vector3d(
		    global_path_msg->pva[0].pos.position.x,
		    global_path_msg->pva[0].pos.position.y,
		    global_path_msg->pva[0].pos.position.z
        );

        nextVelocity_ = Eigen::Vector3d(
		    global_path_msg->pva[0].vel.linear.x,
		    global_path_msg->pva[0].vel.linear.y,
		    global_path_msg->pva[0].vel.linear.z
        );

        nextAcceleration_ = Eigen::Vector3d(
		    global_path_msg->pva[0].acc.linear.x,
		    global_path_msg->pva[0].acc.linear.y,
		    global_path_msg->pva[0].acc.linear.z
        );

		//estimated steps to next waypoint
        Eigen::Vector3d dt = (nextWaypoint_ - currentPose_)/vmax_for_timing;
		stepsToNextWaypoint = ceil(dt.lpNorm<Eigen::Infinity>() / dt_default);
//		ROS_INFO("dtx %f  dty %f  dtz %f  problemelement %f",dt(0),dt(1),dt(2),nextWaypoint_(2)-currentPose_(2));

		//print update to user
		ROS_INFO("Moving to waypoint %f %f %f in %d steps", nextWaypoint_(0),
					nextWaypoint_(1), nextWaypoint_(2), stepsToNextWaypoint);
	}
    else
	{
		nextWaypoint_=currentPose_;
	}
}


void waypointControl::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	double tempTime=(msg->header.stamp).toSec();
	if(tempTime > waypointTime)	//if an update arrives late, ignore it
	{
		waypointTime = tempTime;

		//read in next waypoint and offset by distance between origin and center
        this->nextWaypoint_ = Eigen::Vector3d(
		    msg->pose.position.x,
		    msg->pose.position.y,
		    msg->pose.position.z
        ) + arenaCenter;

		nextVelocity_.setZero();
		nextAcceleration_.setZero();
		oldPose_ = currentPose_;
		oldVelocity_ = currentVelocity_;
		oldAcceleration_.setZero();
        errIntegral.setZero();
	}
}


double waypointControl::saturationF(double &xval, const double satbound)
{
	if(xval > satbound){		
		xval = satbound;
	}
    else if(xval < -satbound){
		xval = -satbound;
	}
}


void waypointControl::limitAcceleration(const Eigen::Vector3d &vv, Eigen::Vector3d &uu)
{
	Eigen::Vector3d velWithAccel;
	double aCheck;
	velWithAccel=vv+dt_default*uu;

	for(int i=0; i<3; i++)
	{
		if(velWithAccel(i) > vmax(i)) //handles positive rel speed
		{
			aCheck = (vmax(i) - velWithAccel(i)) / dt_default;
			if(abs(aCheck) > amax(i))
			{
				uu(i) = abs(aCheck) / aCheck * amax(i); //sign(a) * a_max
			}
            else
			{
				uu(i) = aCheck; //do not use limit if unnecessary
			}
		}
        else if(velWithAccel(i) < -vmax(i)) //handles negative rel speed
		{
			aCheck = (-vmax(i) - velWithAccel(i)) / dt_default;
			if(abs(aCheck) > amax(i))
			{
				uu(i) = abs(aCheck) / aCheck * amax(i); //sign(a) * a_max
			}
            else
			{
				uu(i) = aCheck; //don't unnecessarily limit acceleration
			}
		} //else //all good
	}
}


void waypointControl::checkArrival(const Eigen::Vector3d &cPose)
{
	if(!global_path_msg) return;

    Eigen::Vector3d thisWaypoint = Eigen::Vector3d( 
	    global_path_msg->pva[waypointCounter].pos.position.x,
	    global_path_msg->pva[waypointCounter].pos.position.y,
	    global_path_msg->pva[waypointCounter].pos.position.z
    ) + arenaCenter;

    /* BEGIN CHECK THIS */
	// double thisDist=sqrt(pow(thisWaypoint(0)-cPose(0),2) + pow(thisWaypoint(1)-cPose(1),2) + pow(thisWaypoint(2)-cPose(2),2));
    double thisDist = (thisWaypoint - cPose).norm();
    /* END CHECK THIS */

//	ROS_INFO("distcurr = %f",thisDist-hitDist);
	if(thisDist < hitDist)
	{
//		t0=time0;
		if(waypointCounter < waypointListLen - 1)
		{
			waypointCounter++;

            // Update the old waypoint info
			this->oldPose_          = this->nextWaypoint_;
			this->oldVelocity_      = this->nextVelocity_;
			this->oldAcceleration_  = this->nextAcceleration_;

            /* Update the new waypoint info */
            this->nextWaypoint_ = Eigen::Vector3d(
			    global_path_msg->pva[waypointCounter].pos.position.x,
			    global_path_msg->pva[waypointCounter].pos.position.y,
		        global_path_msg->pva[waypointCounter].pos.position.z
            ) + arenaCenter;

            this->nextVelocity_ = Eigen::Vector3d(
		        global_path_msg->pva[waypointCounter].vel.linear.x,
			    global_path_msg->pva[waypointCounter].vel.linear.y,
			    global_path_msg->pva[waypointCounter].vel.linear.z
            );

            this->nextAcceleration_ = Eigen::Vector3d(
                global_path_msg->pva[waypointCounter].acc.linear.x,
			    global_path_msg->pva[waypointCounter].acc.linear.y,
			    global_path_msg->pva[waypointCounter].acc.linear.z
            );

			//if you have the next time, find segment chunks that correspond to it. Otherwise, guesstimate.
			if(global_path_msg->pva[waypointCounter].header.stamp.toSec() > 1e-4 )
			{
				dtNextWaypoint=(global_path_msg->pva[waypointCounter].header.stamp).toSec() -
									 (global_path_msg->pva[waypointCounter-1].header.stamp).toSec()
									 -0.001;  //subtracting 0.001 to handle roundoff in minimum snap node			
			}
            else
			{
				//guesstimate timing
                this->dtNextWaypoint =  ((nextWaypoint_ - cPose).cwiseQuotient(vmax)).lpNorm<Eigen::Infinity>();
			}
			stepsToNextWaypoint = ceil(dtNextWaypoint/dt_default);
			stepCounter = 0;

            errIntegral.setZero();
			ROS_INFO("Waypoint reached, moving to waypoint %f\t%f\t%f",nextWaypoint_(0),nextWaypoint_(1),nextWaypoint_(2));
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


