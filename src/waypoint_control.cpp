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
      numPathsSoFar(0),
      arrivalModeFlag(0)
{
    this->readROSParameters();

    errIntegral.setZero();

	this->dt_default= 1.0 / this->gpsfps;

	PI=std::atan(1.0)*4;

	// Initialize publishers and subscriber
	//advertise on message topic specified in input file.	USE px4_control/PVA_Ref WITH MARCELINO'S CONTROLLER
	pvaRef_pub_ = nh.advertise<mg_msgs::PVA>(publishtopicname, 10);
	ROS_INFO("Publisher created on topic %s",publishtopicname.c_str());
	pose_sub_ = nh.subscribe(quadPoseTopic, 10, &waypointControl::poseCallback, this, ros::TransportHints().tcpNoDelay());
	waypoint_sub_ = nh.subscribe(quadWaypointTopic,10,&waypointControl::waypointCallback, this, ros::TransportHints().tcpNoDelay());
	waypointList_sub_ = nh.subscribe(quadWaypointListTopic,10,&waypointControl::waypointListCallback, this, ros::TransportHints().tcpNoDelay());
	ROS_INFO("Subscribers created.");
	ROS_INFO("REMEMBER TO CHANGE TO LOCAL POSE MODE");

//	//get initial pose
	ROS_INFO("Waiting for first position measurement...");
	initPose_ = ros::topic::waitForMessage<nav_msgs::Odometry>(quadPoseTopic);
	ROS_INFO("Initial position: %f\t%f\t%f", initPose_->pose.pose.position.x, initPose_->pose.pose.position.y, initPose_->pose.pose.position.z);

	timerPub_ = nh.createTimer(ros::Duration(1.0/pubRate_), &waypointControl::timerCallback, this, false);

//	next_wpt(0)=initPose_->pose.pose.position.x;
//	next_wpt(1)=initPose_->pose.pose.position.y;
//	next_wpt(2)=initPose_->pose.pose.position.z+1; //1m above initial pose
	// Go to 0,0,1 
	//this->nextWaypoint_ = this->arenaCenter + Eigen::Vector3d(0,0,1);
	// Go to 1m above initPose
	this->nextWaypoint_ = this->arenaCenter + Eigen::Vector3d(0,0,1) +
		Eigen::Vector3d(initPose_->pose.pose.position.x, initPose_->pose.pose.position.y, initPose_->pose.pose.position.z);
}


void waypointControl::readROSParameters() 
{
    // Topic names
	ros::param::get("waypoint_control_node/quadPoseTopic", quadPoseTopic);
	ros::param::get("waypoint_control_node/quadWaypointTopic", quadWaypointTopic);
	ros::param::get("waypoint_control_node/quadWaypointListTopic", quadWaypointListTopic);
	ros::param::get("waypoint_control_node/joyTopic", joyTopic);
	ros::param::get("waypoint_control_node/publishPVA_Topic", publishtopicname);
	ros::param::get("waypoint_control_node/defaultMode",default_mode);
	ros::param::get("waypoint_control_node/pubRate", pubRate_);

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
	ros::param::get("waypoint_control_node/kfx", kf_ff(0));
	ros::param::get("waypoint_control_node/kfy", kf_ff(1));
	ros::param::get("waypoint_control_node/kfz", kf_ff(2));
	ros::param::get("waypoint_control_node/maxInteg_X", eImax(0));
	ros::param::get("waypoint_control_node/maxInteg_Y", eImax(1));
	ros::param::get("waypoint_control_node/maxInteg_Z", eImax(2));
	ros::param::get("waypoint_control_node/mass", quadMass);

	// Parameters of px4_control
	ros::param::get("waypoint_control_node/kpX_p", kp_pos(0));
	ros::param::get("waypoint_control_node/kdX_p", kd_pos(0));
	ros::param::get("waypoint_control_node/kiX_p", ki_pos(0));
	ros::param::get("waypoint_control_node/kpY_p", kp_pos(1));
	ros::param::get("waypoint_control_node/kdY_p", kd_pos(1));
	ros::param::get("waypoint_control_node/kiY_p", ki_pos(1));
	ros::param::get("waypoint_control_node/kpZ_p", kp_pos(2));
	ros::param::get("waypoint_control_node/kdZ_p", kd_pos(2));
	ros::param::get("waypoint_control_node/kiZ_p", ki_pos(2));
	ros::param::get("waypoint_control_node/maxInteg_X_p", eImax_pos(0));
	ros::param::get("waypoint_control_node/maxInteg_Y_p", eImax_pos(1));
	ros::param::get("waypoint_control_node/maxInteg_Z_p", eImax_pos(2));

	// Parameters of px4_control
	ros::param::get("waypoint_control_node/kpX_h", kp_hov(0));
	ros::param::get("waypoint_control_node/kdX_h", kd_hov(0));
	ros::param::get("waypoint_control_node/kiX_h", ki_hov(0));
	ros::param::get("waypoint_control_node/kpY_h", kp_hov(1));
	ros::param::get("waypoint_control_node/kdY_h", kd_hov(1));
	ros::param::get("waypoint_control_node/kiY_h", ki_hov(1));
	ros::param::get("waypoint_control_node/kpZ_h", kp_hov(2));
	ros::param::get("waypoint_control_node/kdZ_h", kd_hov(2));
	ros::param::get("waypoint_control_node/kiZ_h", ki_hov(2));
	ros::param::get("waypoint_control_node/maxInteg_X_h", eImax_hov(0));
	ros::param::get("waypoint_control_node/maxInteg_Y_h", eImax_hov(1));
	ros::param::get("waypoint_control_node/maxInteg_Z_h", eImax_hov(2));

	// Safety parameters. Only currently in use to make intermediate points when initalizing
	ros::param::get("waypoint_control_node/accel_max", max_accel);

	// misc parameters
	ros::param::get("waypoint_control_node/gps_fps", gpsfps);
	ros::param::get("waypoint_control_node/waypointHitDist", hitDist);
	ros::param::get("waypoint_control_node/xCenter", arenaCenter(0));
	ros::param::get("waypoint_control_node/yCenter", arenaCenter(1));
	ros::param::get("waypoint_control_node/zCenter", arenaCenter(2));
	ros::param::get("waypoint_control_node/vmax_for_timing",vmax_for_timing);
	ros::param::get("waypoint_control_node/vmax_real",vmax_real);

}


void waypointControl::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odomMsg_ = msg;
}


void waypointControl::timerCallback(const ros::TimerEvent &event)
{ 
	double tcurr = (ros::Time::now()).toSec();
	if(tcurr - t_thispt > dt_nextpt)
	{


	static const Eigen::Vector3d initLocationVec =
			Eigen::Vector3d(initPose_->pose.pose.position.x, initPose_->pose.pose.position.y,
			initPose_->pose.pose.position.z);

	//PID3 structure can also be used here
    this->currentPose_ = Eigen::Vector3d(
	    odomMsg_->pose.pose.position.x,
	    odomMsg_->pose.pose.position.y,
	    odomMsg_->pose.pose.position.z
    );

    this->currentVelocity_ = Eigen::Vector3d(
	    odomMsg_->twist.twist.linear.x,
	    odomMsg_->twist.twist.linear.y,
	    odomMsg_->twist.twist.linear.z
    );


    int tryTemp=1;

    //Prepare for multiple cases
   	double substep, vmaxToUse;

	//if there is a path, follow it.  If not, hover near arena center
    if(global_path_msg)
    {
    	if(waypointCounter<1)
    	{vmaxToUse=vmax_for_timing;}
    	else{vmaxToUse=vmax_real;}

    	if(hitDist>1e-2)
    	{
			checkArrival(currentPose_);
		}else{
			updateArrivalTiming(currentPose_);
		}
//		limitAcceleration(currentVelocity_,uPID);

		//Move to next waypoint in multiple substeps
		//Check to see if the window needs to increase in size
 	    Eigen::Vector3d dt = (nextWaypoint_ - currentPose_)/vmaxToUse;  //normally use vmax_real here
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
		//ROS_INFO("Substep: %f",substep);
	}else  //if no message has been received
	{
		//set next wpt to be stationary at arena center
		//nextWaypoint_=arenaCenter; //desired init pose
		//nextWaypoint_(2)=arenaCenter(2) + 1; //behavior of "take off and hover at 1m"
		nextWaypoint_ = initLocationVec+Eigen::Vector3d(0,0,1);
		nextVelocity_.setZero();
		nextAcceleration_.setZero();
		oldPose_=currentPose_;
//		oldVelocity_=currentVelocity_;
		oldVelocity_.setZero();
		oldAcceleration_.setZero();

		nextYaw_=0.0;

// 	    Eigen::Vector3d dt = (nextWaypoint_ - currentPose_).cwiseQuotient(vmax);
 	    Eigen::Vector3d dt = (nextWaypoint_ - oldPose_)/vmax_for_timing;
		double estStepsMaxVel = std::ceil(dt.lpNorm<Eigen::Infinity>() / dt_default);
		substep=1-1/estStepsMaxVel;
		if(estStepsMaxVel<=2.1)  //forces origin since dt is almost always ~1.1 timesteps near origin
		{
			substep=0;
		}
		//ROS_INFO("Substep: %f",substep);
	}

	//uses the PVA message from px4_control package
	mg_msgs::PVA PVA_Ref_msg;
	PVA_Ref_msg.yaw = nextYaw_;	//can try nonzero if optimal
//	std::cout<<nextYaw_<<std::endl;
	
	//fill message fields
	//The proper way to do this is with discrete integration but handling it with substeps is close enough
    Eigen::Vector3d tmp, tmp2, uPID;


	//Working code that uses the underlying PID in px4_control.
	tmp = nextWaypoint_ - substep * (nextWaypoint_ - oldPose_);
    tmp2 = tmp-oldPose_;
 //	ROS_INFO("x: %f  y: %f  z: %f",tmp2(0),tmp2(1),tmp2(2));
	PVA_Ref_msg.Pos.x = tmp(0);
	PVA_Ref_msg.Pos.y = tmp(1);
	PVA_Ref_msg.Pos.z = tmp(2);

	tmp = nextVelocity_ - substep * (nextVelocity_ - oldVelocity_);
	if(tmp.norm()>vmax_for_timing)
	{tmp=tmp*vmax_for_timing/tmp.norm();}
	PVA_Ref_msg.Vel.x=tmp(0);
	PVA_Ref_msg.Vel.y=tmp(1);
	PVA_Ref_msg.Vel.z=tmp(2);

	//px4_control uses accelerations for full FF reference
	tmp = (nextAcceleration_ - substep * (nextAcceleration_ - oldAcceleration_));
	if(tmp.norm()>max_accel)
	{tmp=tmp*max_accel/tmp.norm();}
	PVA_Ref_msg.Acc.x = kf_ff(0)*tmp(0);
	PVA_Ref_msg.Acc.y = kf_ff(1)*tmp(1);
	PVA_Ref_msg.Acc.z = kf_ff(2)*tmp(2);
	

	pvaRef_pub_.publish(PVA_Ref_msg);
	}
}


void waypointControl::waypointListCallback(const mg_msgs::PVAYStampedTrajectory::ConstPtr &msg)
{
  std::cout << "got something" << std::endl;
	//int nn= ;//length of pose list
	numPathsSoFar++;
	arrivalModeFlag=0;
	if(msg) {
		dt_nextpt = (msg->trajectory[1].header.stamp).toSec() - (msg->trajectory[0].header.stamp).toSec();
		t_thispt = (ros::Time::now()).toSec();
		std::cout << "Trajectory received" << std::endl;
		waypointListLen=msg->trajectory.size();
		global_path_msg = msg;

    std::cout << "total points: " << waypointListLen << std::endl;

		//initialize waypoint counter
		waypointCounter=0;
		counter++; //total number of waypoint list/paths received

		//set old states to 0
		oldPose_ = currentPose_;
		oldVelocity_ = currentVelocity_;
        oldAcceleration_.setZero();

		//reset PID params
        errIntegral.setZero();

		//get next waypoint trajectory from list
        nextWaypoint_ = Eigen::Vector3d(
		    global_path_msg->trajectory[0].pos.x,
		    global_path_msg->trajectory[0].pos.y,
		    global_path_msg->trajectory[0].pos.z
        );

        nextVelocity_ = Eigen::Vector3d(
		    global_path_msg->trajectory[0].vel.linear.x,
		    global_path_msg->trajectory[0].vel.linear.y,
		    global_path_msg->trajectory[0].vel.linear.z
        );

        nextAcceleration_ = Eigen::Vector3d(
		    global_path_msg->trajectory[0].acc.linear.x,
		    global_path_msg->trajectory[0].acc.linear.y,
		    global_path_msg->trajectory[0].acc.linear.z
        );

		//estimated steps to next waypoint
        Eigen::Vector3d dt = (nextWaypoint_ - currentPose_)/vmax_for_timing;
		stepsToNextWaypoint = ceil(dt.lpNorm<Eigen::Infinity>() / dt_default);
//		ROS_INFO("dtx %f  dty %f  dtz %f  problemelement %f",dt(0),dt(1),dt(2),nextWaypoint_(2)-currentPose_(2));

		//print update to user
		//ROS_INFO("Moving to waypoint %f %f %f in %d steps", nextWaypoint_(0),
		//			nextWaypoint_(1), nextWaypoint_(2), stepsToNextWaypoint);
		ROS_INFO("Trajectory received.");
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


Eigen::Vector3d waypointControl::vectorSaturationF(Eigen::Vector3d &vec1, const Eigen::Vector3d &vecSatbound)
{
	for(int i=0; i<3; i++)
	{
		if(vec1(i) > vecSatbound(i)){
			vec1(i) = vecSatbound(i);
		}else if(vec1(i) < -1*vecSatbound(i)){
			vec1(i) = -1*vecSatbound(i);
		}
	}
}


void waypointControl::limitAcceleration(const Eigen::Vector3d &vv, Eigen::Vector3d &uu)
{}


void waypointControl::checkArrival(const Eigen::Vector3d &cPose)
{
	if(!global_path_msg) return;

    Eigen::Vector3d thisWaypoint = Eigen::Vector3d( 
	    global_path_msg->trajectory[waypointCounter].pos.x,
	    global_path_msg->trajectory[waypointCounter].pos.y,
	    global_path_msg->trajectory[waypointCounter].pos.z
    ) + arenaCenter;

    /* BEGIN CHECK THIS */
	// double thisDist=sqrt(pow(thisWaypoint(0)-cPose(0),2) + pow(thisWaypoint(1)-cPose(1),2) + pow(thisWaypoint(2)-cPose(2),2));
    double thisDist = (thisWaypoint - cPose).norm();
    /* END CHECK THIS */

//	ROS_INFO("distcurr = %f",thisDist-hitDist);
	if(thisDist < hitDist)
	{
		dt_nextpt = (global_path_msg->trajectory[1].header.stamp).toSec() - (global_path_msg->trajectory[0].header.stamp).toSec();
		t_thispt = (ros::Time::now()).toSec();		
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
			    global_path_msg->trajectory[waypointCounter].pos.x,
			    global_path_msg->trajectory[waypointCounter].pos.y,
		        global_path_msg->trajectory[waypointCounter].pos.z
            ) + arenaCenter;

            this->nextVelocity_ = Eigen::Vector3d(
		        global_path_msg->trajectory[waypointCounter].vel.linear.x,
			    global_path_msg->trajectory[waypointCounter].vel.linear.y,
			    global_path_msg->trajectory[waypointCounter].vel.linear.z
            );

            this->nextAcceleration_ = Eigen::Vector3d(
                global_path_msg->trajectory[waypointCounter].acc.linear.x,
			    global_path_msg->trajectory[waypointCounter].acc.linear.y,
			    global_path_msg->trajectory[waypointCounter].acc.linear.z
            );

			//if you have the next time, find segment chunks that correspond to it. Otherwise, guesstimate.
			if(global_path_msg->trajectory[waypointCounter].header.stamp.toSec() > 1e-4 )
			{
				dtNextWaypoint=(global_path_msg->trajectory[waypointCounter].header.stamp).toSec() -
									 (global_path_msg->trajectory[waypointCounter-1].header.stamp).toSec()
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
		}else
		{
			if(arrivalModeFlag==0) //only print arrival message once
			{
				ROS_INFO("Final waypoint reached, holding station.");
			}
			arrivalModeFlag=1;

			//Set old VA to 0 to hold station
			nextVelocity_.setZero();
			nextAcceleration_.setZero();
			oldVelocity_.setZero();
			oldAcceleration_.setZero();
		}
	}
}

void waypointControl::updateArrivalTiming(const Eigen::Vector3d &cPose)
{
	if(!global_path_msg) return;

	if(stepCounter>=stepsToNextWaypoint)
	{
		dt_nextpt = (global_path_msg->trajectory[1].header.stamp).toSec() - (global_path_msg->trajectory[0].header.stamp).toSec();
		t_thispt = (ros::Time::now()).toSec();
		//if not at endpoint
		if(waypointCounter < waypointListLen - 1)
		{
			waypointCounter++;

            // Update the old waypoint info
			this->oldPose_          = this->nextWaypoint_;
			this->oldVelocity_      = this->nextVelocity_;
			this->oldAcceleration_  = this->nextAcceleration_;

            /* Update the new waypoint info */
            this->nextWaypoint_ = Eigen::Vector3d(
			    global_path_msg->trajectory[waypointCounter].pos.x,
			    global_path_msg->trajectory[waypointCounter].pos.y,
		        global_path_msg->trajectory[waypointCounter].pos.z
            );

            this->nextVelocity_ = Eigen::Vector3d(
		        global_path_msg->trajectory[waypointCounter].vel.linear.x,
			    global_path_msg->trajectory[waypointCounter].vel.linear.y,
			    global_path_msg->trajectory[waypointCounter].vel.linear.z
            );

            this->nextAcceleration_ = Eigen::Vector3d(
                global_path_msg->trajectory[waypointCounter].acc.linear.x,
			    global_path_msg->trajectory[waypointCounter].acc.linear.y,
			    global_path_msg->trajectory[waypointCounter].acc.linear.z
            );

            
            if(this->nextYaw_<0)
            	{this->nextYaw_=this->nextYaw_+2*PI;}

			//if you have the next time, find segment chunks that correspond to it. Otherwise, guesstimate.
			if(global_path_msg->trajectory[waypointCounter].header.stamp.toSec() > 1e-4 )
			{
				dtNextWaypoint=(global_path_msg->trajectory[waypointCounter].header.stamp).toSec() -
									 (global_path_msg->trajectory[waypointCounter-1].header.stamp).toSec()
									 -0.001;  //subtracting 0.001 to handle roundoff in minimum snap node			
			}
            else
			{
				//guesstimate timing
                this->dtNextWaypoint =  ((nextWaypoint_ - cPose)/vmax_for_timing).lpNorm<Eigen::Infinity>();
			}
			stepsToNextWaypoint = ceil(dtNextWaypoint/dt_default);
			stepCounter = 0;

            errIntegral.setZero();
            //print status
            ROS_INFO("Waypoint time reached, moving to waypoint %f\t%f\t%f in %d steps",nextWaypoint_(0),nextWaypoint_(1),nextWaypoint_(2), stepsToNextWaypoint);
		}else
		{
			if(arrivalModeFlag==0) //only print arrival message once
			{
				ROS_INFO("Final waypoint reached, holding station.");
			}
			arrivalModeFlag=1;

			//Set old VA to 0 to hold station
			nextVelocity_.setZero();
			nextAcceleration_.setZero();
			oldVelocity_.setZero();
			oldAcceleration_.setZero();
		}
	}
}







