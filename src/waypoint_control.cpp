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
	joy_sub_ = nh.subscribe(joyTopic,10,&waypointControl::joyCallback, this, ros::TransportHints().tcpNoDelay());
	controlParamUpdate = nh.serviceClient<px4_control::updatePx4param>("/px4_control_node/updatePosControlParam");
	ROS_INFO("Subscribers created.");
	ROS_INFO("REMEMBER TO CHANGE TO LOCAL POSE MODE");

//	//get initial pose
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
	ros::param::get("waypoint_control_node/quadWaypointTopic", quadWaypointTopic);
	ros::param::get("waypoint_control_node/quadWaypointListTopic", quadWaypointListTopic);
	ros::param::get("waypoint_control_node/joyTopic", joyTopic);
	ros::param::get("waypoint_control_node/publishPVA_Topic", publishtopicname);
	ros::param::get("waypoint_control_node/defaultMode",default_mode);

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
	ros::param::get("waypoint_control_node/vx_max", vmax(0));
	ros::param::get("waypoint_control_node/vy_max", vmax(1));
	ros::param::get("waypoint_control_node/vz_max", vmax(2));
	ros::param::get("waypoint_control_node/ax_max", max_accel(0));
	ros::param::get("waypoint_control_node/ay_max", max_accel(1));
	ros::param::get("waypoint_control_node/az_max", max_accel(2));

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


    int tryTemp=1;

    //Prepare for multiple cases
   	double substep, vmaxToUse;

	//if there is a path, follow it.  If not, hover near arena center
    if(global_path_msg)
    {
    	if(waypointCounter<1)
    	{vmaxToUse=vmax_for_timing;}
    	else{vmaxToUse=vmax_real;}

    	if(hitDist>0.001)
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
	}else  //if no message has been received
	{
		//set next wpt to be stationary at arena center
		nextWaypoint_=arenaCenter; //desired init pose
		//nextWaypoint_(2)=arenaCenter(2) + 1; //behavior of "take off and hover at 1m" REMOVED
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
//		ROS_INFO("Substep: %f",substep);
	}

	//uses the PVA message from px4_control package
	px4_control::PVA PVA_Ref_msg;
	PVA_Ref_msg.yaw = nextYaw_;	//can try nonzero if optimal
	
	//fill message fields
	//The proper way to do this is with discrete integration but handling it with substeps is close enough
    Eigen::Vector3d tmp, tmp2, uPID;

/*
    //Uses PID controller unique to waypoint_control
    PVA_Ref_msg.Pos.x = currentPose_(0); //send errors of 0 to px4_control, handle everything via feedforward
    PVA_Ref_msg.Pos.y = currentPose_(1);
    PVA_Ref_msg.Pos.z = currentPose_(2);
    PVA_Ref_msg.Vel.x = currentVelocity_(0); //send 0 error
    PVA_Ref_msg.Vel.y = currentVelocity_(1);
    PVA_Ref_msg.Vel.z = currentVelocity_(2);

    //tmp refers to e, edot for this controller
    tmp = (nextWaypoint_ - substep * (nextWaypoint_ - oldPose_)) - currentPose_;
    errIntegral = errIntegral + dt_default*tmp;
    errIntegral = vectorSaturationF(errIntegral, eImax);
    tmp2 = -1.0*currentVelocity_;
    //construct PID to send via FF
    uPID = (1.0/quadMass)*(kp.cwiseProduct(tmp) + kd.cwiseProduct(tmp2) + ki.cwiseProduct(errIntegral));
    uPID = vectorSaturationF(uPID,max_accel);
    //note: px4_control multiplies FF by mass but does not multiply PID by mass so we have to cancel it.
    PVA_Ref_msg.Acc.x = uPID(0);
    PVA_Ref_msg.Acc.y = uPID(1);
    PVA_Ref_msg.Acc.z = uPID(2);
*/


/*
	//Proper feedforward terms with valid references.  Does not handle noise well.
    tmp = nextVelocity_ - substep * (nextVelocity_ - oldVelocity_);
	PVA_Ref_msg.Vel.x = tmp(0);
	PVA_Ref_msg.Vel.y = tmp(1);
	PVA_Ref_msg.Vel.z = tmp(2);
*/


	//Working code that uses the underlying PID in px4_control.
	tmp = nextWaypoint_ - substep * (nextWaypoint_ - oldPose_);
    tmp2 = tmp-oldPose_;
 //	ROS_INFO("x: %f  y: %f  z: %f",tmp2(0),tmp2(1),tmp2(2));
	PVA_Ref_msg.Pos.x = tmp(0);
	PVA_Ref_msg.Pos.y = tmp(1);
	PVA_Ref_msg.Pos.z = tmp(2);

	tmp = nextVelocity_ - substep * (nextVelocity_ - oldVelocity_);
	PVA_Ref_msg.Vel.x=tmp(0);
	PVA_Ref_msg.Vel.y=tmp(1);
	PVA_Ref_msg.Vel.z=tmp(2);

	//px4_control uses accelerations for full FF reference
	tmp = (nextAcceleration_ - substep * (nextAcceleration_ - oldAcceleration_));
	PVA_Ref_msg.Acc.x = kf_ff(0)*tmp(0);
	PVA_Ref_msg.Acc.y = kf_ff(1)*tmp(1);
	PVA_Ref_msg.Acc.z = kf_ff(2)*tmp(2);
	
	/*
	PVA_Ref_msg.Vel.x=0;
	PVA_Ref_msg.Vel.y=0;
	PVA_Ref_msg.Vel.z=0;
	PVA_Ref_msg.Acc.x=0;
	PVA_Ref_msg.Acc.y=0;
	PVA_Ref_msg.Acc.z=0;
	*/


/*
	//Discrete time optimal FF gain (too aggressive for stochastic use)
	Eigen::MatrixXd A_tr(6,6), Hx(3,6), B_tr(6,3), K_tr(3,6), eye6(6,6), N_tr(3,3);
	A_tr=Eigen::MatrixXd::Identity(6,6);
	A_tr.topRightCorner(3, 3) = dt_default*Eigen::MatrixXd::Identity(3,3);
	Hx<<1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0;
	B_tr<<dt_default, 0, 0,
		0, dt_default, 0,
		0, 0, dt_default,
		dt_default*dt_default/2, 0, 0,
		0, dt_default*dt_default/2, 0,
		0, 0, dt_default*dt_default/2;
	eye6=Eigen::MatrixXd::Identity(6,6);
	K_tr<<kp(0), 0, 0, kd(0), 0, 0,
		0, kp(1), 0, 0, kd(1), 0,
		0, 0, kp(2), 0, 0, kd(2);
	N_tr = (-Hx*(A_tr - B_tr*K_tr - eye6).inverse()*B_tr).inverse(); 
	ROS_INFO("%f %f %f \n %f %f %f \n %f %f %f", N_tr(0,0), N_tr(0,1), N_tr(0,2), N_tr(1,0), N_tr(1,1),
		N_tr(1,2), N_tr(2,0), N_tr(2,1), N_tr(2,2));
*/

/*
	//"move" reference to model controller internally
	//NOTE: MUST DISABLE INTEGRAL ACTION IN px4_control 
	Eigen::Vector3d aFake, uDes;
	Eigen::MatrixXd A_tr(6,6), Hx(3,6), xPV(6,1);
	aFake=nextAcceleration_ - substep * (nextAcceleration_ - oldAcceleration_);

	//fill xPV (full x-state)
	for(int ij=0; ij<3; ij++)
	{
		xPV(ij)=currentPose_(ij);
		xPV(ij+3)=currentVelocity_(ij);
	}
	Hx<<1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0;

	A_tr=Eigen::MatrixXd::Identity(6,6);
	A_tr.topRightCorner(3, 3)    = dt_default*Eigen::MatrixXd::Identity(3,3);
	tmp = nextWaypoint_ - substep * (nextWaypoint_ - oldPose_);
	Eigen::Vector3d dx_des=tmp-Hx*A_tr*xPV;  //outputs desired delta_x
	uDes=(dx_des*2.0/dt_default/dt_default).cwiseQuotient(kp);

	PVA_Ref_msg.Pos.x=currentPose_(0)+uDes(0);
	PVA_Ref_msg.Pos.y=currentPose_(1)+uDes(1);
	PVA_Ref_msg.Pos.z=currentPose_(2)+uDes(2);
	PVA_Ref_msg.Vel.x=currentVelocity_(0);
	PVA_Ref_msg.Vel.y=currentVelocity_(1);
	PVA_Ref_msg.Vel.z=currentVelocity_(2);
	PVA_Ref_msg.Acc.x=aFake(0);
	PVA_Ref_msg.Acc.y=aFake(1);
	PVA_Ref_msg.Acc.z=aFake(2);
*/

	pvaRef_pub_.publish(PVA_Ref_msg);
}


void waypointControl::waypointListCallback(const app_pathplanner_interface::PVATrajectory::ConstPtr &msg)
{
	//int nn= ;//length of pose list
	numPathsSoFar++;
	arrivalModeFlag=0;
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


void waypointControl::joyCallback(const sensor_msgs::Joy &msg)
{
	if(msg.buttons[1]==1) //pressed B
	{
		//ROS_INFO("service called");
		px4_control::updatePx4param param_srv;
		param_srv.request.data.resize(10);
		param_srv.request.data={kp(0),kp(1),kp(2),kd(0),kd(1),kd(2),ki(0),ki(1),ki(2),
			eImax(0),eImax(1),eImax(2)};
		controlParamUpdate.call(param_srv);
		
	}else if(msg.buttons[2]==1)  //pressed X
	{
		if(default_mode.compare("aggressive")==0)
		{
			//ROS_INFO("service called");
			px4_control::updatePx4param param_srv;
			param_srv.request.data.resize(10);
			param_srv.request.data={kp_pos(0),kp_pos(1),kp_pos(2),kd_pos(0),kd_pos(1),kd_pos(2),
				ki_pos(0),ki_pos(1),ki_pos(2),eImax_pos(0),eImax_pos(1),eImax_pos(2)};
			controlParamUpdate.call(param_srv);
		}else
		{
			px4_control::updatePx4param param_srv;
			param_srv.request.data.resize(10);
			param_srv.request.data={kp_hov(0),kp_hov(1),kp_hov(2),kd_hov(0),kd_hov(1),kd_hov(2),
				ki_hov(0),ki_hov(1),ki_hov(2),eImax_hov(0),eImax_hov(1),eImax_hov(2)};
			controlParamUpdate.call(param_srv);
		}
	}else if(msg.axes[7]>=0.9) //pressed UP on dpad
	{
		px4_control::updatePx4param param_srv;
		param_srv.request.data.resize(10);
		param_srv.request.data={kp_hov(0),kp_hov(1),kp_hov(2),kd_hov(0),kd_hov(1),kd_hov(2),
			ki_hov(0),ki_hov(1),ki_hov(2),eImax_hov(0),eImax_hov(1),eImax_hov(2)};
		controlParamUpdate.call(param_srv);
	}else if(msg.axes[7]<=-0.9) //pressed DOWN on dpad
	{
		//ROS_INFO("service called");
		px4_control::updatePx4param param_srv;
		param_srv.request.data.resize(10);
		param_srv.request.data={kp_pos(0),kp_pos(1),kp_pos(2),kd_pos(0),kd_pos(1),kd_pos(2),
			ki_pos(0),ki_pos(1),ki_pos(2),eImax_pos(0),eImax_pos(1),eImax_pos(2)};
		controlParamUpdate.call(param_srv);
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
{
	Eigen::Vector3d velWithAccel;
	double aCheck;
	velWithAccel=vv+dt_default*uu;

	for(int i=0; i<3; i++)
	{
		if(velWithAccel(i) > vmax(i)) //handles positive rel speed
		{
			aCheck = (vmax(i) - velWithAccel(i)) / dt_default;
			if(abs(aCheck) > max_accel(i))
			{
				uu(i) = abs(aCheck) / aCheck * max_accel(i); //sign(a) * a_max
			}
            else
			{
				uu(i) = aCheck; //do not use limit if unnecessary
			}
		}
        else if(velWithAccel(i) < -vmax(i)) //handles negative rel speed
		{
			aCheck = (-vmax(i) - velWithAccel(i)) / dt_default;
			if(abs(aCheck) > max_accel(i))
			{
				uu(i) = abs(aCheck) / aCheck * max_accel(i); //sign(a) * a_max
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
			    global_path_msg->pva[waypointCounter].pos.position.x,
			    global_path_msg->pva[waypointCounter].pos.position.y,
		        global_path_msg->pva[waypointCounter].pos.position.z
            );

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

            //this could be done without filling the quaternion but this leaves more data for later
            Eigen::Quaternionf thisQuat;
            thisQuat.x()=global_path_msg->pva[waypointCounter].pos.orientation.x;
            thisQuat.y()=global_path_msg->pva[waypointCounter].pos.orientation.y;
            thisQuat.z()=global_path_msg->pva[waypointCounter].pos.orientation.z;
            thisQuat.w()=global_path_msg->pva[waypointCounter].pos.orientation.w;
            this->nextYaw_=atan2(2.0*(thisQuat.w()*thisQuat.z() + thisQuat.x()*thisQuat.y()),
            		1.0 - 2.0*(thisQuat.y()*thisQuat.y() + thisQuat.z()*thisQuat.z()));

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







