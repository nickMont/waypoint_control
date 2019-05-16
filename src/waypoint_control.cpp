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

	takeoffHeight_=0;
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
			updateArrivalTiming();
		}
		
		mg_msgs::PVA PVA_Ref_msg;
		PVA_Ref_msg.yaw = nextYaw_;	//can try nonzero if optimal
		//	std::cout<<nextYaw_<<std::endl;
		
		//fill message fields
		//The proper way to do this is with discrete integration but handling it with substeps is close enough
	    Eigen::Vector3d tmp;


		//Working code that uses the underlying PID in px4_control.
		tmp = nextWaypoint_;
	 	//	ROS_INFO("x: %f  y: %f  z: %f",tmp2(0),tmp2(1),tmp2(2));
		PVA_Ref_msg.Pos.x = tmp(0);
		PVA_Ref_msg.Pos.y = tmp(1);
		PVA_Ref_msg.Pos.z = tmp(2);

		tmp = nextVelocity_;
		PVA_Ref_msg.Vel.x=tmp(0);
		PVA_Ref_msg.Vel.y=tmp(1);
		PVA_Ref_msg.Vel.z=tmp(2);

		//px4_control uses accelerations for full FF reference
		tmp = nextAcceleration_;
		PVA_Ref_msg.Acc.x = tmp(0);
		PVA_Ref_msg.Acc.y = tmp(1);
		PVA_Ref_msg.Acc.z = tmp(2);
		

		pvaRef_pub_.publish(PVA_Ref_msg);
		//ROS_INFO("Substep: %f",substep);
	}else  //if no message has been received
	{
		//set next wpt to be stationary at arena center
		//nextWaypoint_=arenaCenter; //desired init pose
		//nextWaypoint_(2)=arenaCenter(2) + 1; //behavior of "take off and hover at 1m"
		nextWaypoint_ = initLocationVec+Eigen::Vector3d(0,0,takeoffHeight_);
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

		mg_msgs::PVA PVA_Ref_msg;
		PVA_Ref_msg.yaw = nextYaw_;	//can try nonzero if optimal
	//	std::cout<<nextYaw_<<std::endl;
		
		//fill message fields
		//The proper way to do this is with discrete integration but handling it with substeps is close enough
	    Eigen::Vector3d tmp, tmp2, uPID;
		tmp = nextWaypoint_ - substep * (nextWaypoint_ - oldPose_);
	    tmp2 = tmp-oldPose_;
		PVA_Ref_msg.Pos.x = tmp(0);
		PVA_Ref_msg.Pos.y = tmp(1);
		PVA_Ref_msg.Pos.z = tmp(2);
		tmp = nextVelocity_ - substep * (nextVelocity_ - oldVelocity_);
		if(tmp.norm()>vmax_for_timing)
		{tmp=tmp*vmax_for_timing/tmp.norm();}
		PVA_Ref_msg.Vel.x=tmp(0);
		PVA_Ref_msg.Vel.y=tmp(1);
		PVA_Ref_msg.Vel.z=tmp(2);
		tmp = (nextAcceleration_ - substep * (nextAcceleration_ - oldAcceleration_));
		if(tmp.norm()>max_accel)
		{tmp=tmp*max_accel/tmp.norm();}
		PVA_Ref_msg.Acc.x = kf_ff(0)*tmp(0);
		PVA_Ref_msg.Acc.y = kf_ff(1)*tmp(1);
		PVA_Ref_msg.Acc.z = kf_ff(2)*tmp(2);
		

		pvaRef_pub_.publish(PVA_Ref_msg);
	}


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
		ROS_INFO("Trajectory received.");
	}
    else
	{
		nextWaypoint_=currentPose_;
	}
}

void waypointControl::updateArrivalTiming()
{
	if(!global_path_msg) return;

	
	t_thispt = (ros::Time::now()).toSec();
	if(waypointCounter < waypointListLen - 1)
	{
		waypointCounter++;
		dt_nextpt = (global_path_msg->trajectory[waypointCounter].header.stamp).toSec() - (global_path_msg->trajectory[waypointCounter-1].header.stamp).toSec();

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




