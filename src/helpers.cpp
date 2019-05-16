#include "waypointcontrol.hpp"

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