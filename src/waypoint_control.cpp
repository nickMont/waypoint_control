#include "ros/ros.h"
#include "waypointcontrol.hpp"
#include <Eigen/Geometry>
#include <string>

namespace waypoint_control
{

waypointControl::waypointControl(ros::NodeHandle &nh)
{
  quadName = ros::this_node::getName();
  ros::param::get("waypoint_control_node/quadPoseTopic", quadPoseTopic);
  ros::param::get("waypoint_control_node/quadWptTopic", quadWptTopic);
  ros::param::get("waypoint_control_node/quadWptListTopic", quadWptListTopic);
  ros::param::get("waypoint_control_node/publishPVA_Topic", publishtopicname);

  //confirm that parameters were read correctly
  ROS_INFO("Preparing pose subscriber on channel %s",quadPoseTopic.c_str());
  ROS_INFO("Preparing waypoint subscriber on channel %s",quadWptTopic.c_str());
  ROS_INFO("Preparing wptList subscriber on channel %s",quadWptListTopic.c_str());


  counter=0;
  poseTime=0.0;
  wptTime=0.0;
  stepCounter=0;

  //read PID params from file
  ros::param::get("waypoint_control_node/kpX",kp(0));
  ros::param::get("waypoint_control_node/kdX",kd(0));
  ros::param::get("waypoint_control_node/kiX",ki(0));
  ros::param::get("waypoint_control_node/kpY",kp(1));
  ros::param::get("waypoint_control_node/kdY",kd(1));
  ros::param::get("waypoint_control_node/kiY",ki(1));
  ros::param::get("waypoint_control_node/kpZ",kp(2));
  ros::param::get("waypoint_control_node/kdZ",kd(2));
  ros::param::get("waypoint_control_node/kiZ",ki(2));
  ros::param::get("waypoint_control_node/maxInteg_X",eImax(0));
  ros::param::get("waypoint_control_node/maxInteg_Y",eImax(1));
  ros::param::get("waypoint_control_node/maxInteg_Z",eImax(2));

  //read in safety params. Only currently in use to make intermediate points when initalizing
  ros::param::get("waypoint_control_node/vx_max",vmax(0));
  ros::param::get("waypoint_control_node/vy_max",vmax(1));
  ros::param::get("waypoint_control_node/vz_max",vmax(2));
  ros::param::get("waypoint_control_node/gps_fps",gpsfps);
  ros::param::get("waypoint_control_node/waypointHitDist",hitDist);
  ros::param::get("waypoint_control_node/ax_max",amax(0));
  ros::param::get("waypoint_control_node/ay_max",amax(1));
  ros::param::get("waypoint_control_node/az_max",amax(2));
  ros::param::get("waypoint_control_node/xCenter",arenaCenter(0));
  ros::param::get("waypoint_control_node/yCenter",arenaCenter(1));
  ros::param::get("waypoint_control_node/zCenter",arenaCenter(2));

  //set default wpt to origin
  if(counter==0)
  {
    next_wpt=arenaCenter;
    next_wpt(2)=next_wpt(2)+1; //take off before doing anything stupid
    numPathsSoFar=0;
  }
  next_vel(0)=0; next_vel(1)=0; next_vel(2)=0;
  next_acc(0)=0; next_acc(1)=0; next_acc(2)=0;
  old_acc(0)=0; old_acc(1)=0; old_acc(2)=0;
  old_vel(0)=0; old_vel(1)=0; old_vel(2)=0;
  old_pose(0)=0; old_pose(1)=0; old_pose(2)=0;

  dt_default=1.0/gpsfps;

  // Initialize publishers and subscriber
  //advertise on message topic specified in input file.  USE px4_control/PVA_Ref WITH MARCELINO'S CONTROLLER
  pvaRef_pub_ = nh.advertise<px4_control::PVA>(publishtopicname, 10);
  ROS_INFO("Publisher created on topic %s",publishtopicname.c_str());
  pose_sub_ = nh.subscribe(quadPoseTopic, 10, &waypointControl::poseCallback,
                            this, ros::TransportHints().tcpNoDelay());
  waypoint_sub_ = nh.subscribe(quadWptTopic,10,&waypointControl::wptCallback,
                            this, ros::TransportHints().tcpNoDelay());
  waypointList_sub_ = nh.subscribe(quadWptListTopic,10,&waypointControl::wptListCallback,
                            this, ros::TransportHints().tcpNoDelay());
  ROS_INFO("Subscribers successfully created.");

  //get initial pose
  ROS_INFO("Waiting for first position measurement...");
  initPose_ = ros::topic::waitForMessage<nav_msgs::Odometry>(quadPoseTopic);
  ROS_INFO("Initial position: %f\t%f\t%f", initPose_->pose.pose.position.x, initPose_->pose.pose.position.y, initPose_->pose.pose.position.z);
//  next_wpt(0)=initPose_->pose.pose.position.x;
//  next_wpt(1)=initPose_->pose.pose.position.y;
//  next_wpt(2)=initPose_->pose.pose.position.z+1; //1m above initial pose
}


void waypointControl::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ //Sends PVA_Ref based on current PVA when received

  //Send new PVA when new KFPose received
  //static ros::Time t_last_proc = msg->header.stamp;
  //double dt = (msg->header.stamp - t_last_proc).toSec();
  //t_last_proc = msg->header.stamp;
  
  //PID3 structure can also be used here
  poseCurr(0)=msg->pose.pose.position.x;
  poseCurr(1)=msg->pose.pose.position.y;
  poseCurr(2)=msg->pose.pose.position.z;
  velCurr(0)=msg->twist.twist.linear.x;
  velCurr(1)=msg->twist.twist.linear.y;
  velCurr(2)=msg->twist.twist.linear.z;
//  poseCurr(0)=0; poseCurr(1)=0; poseCurr(2)=0;
//  velCurr(0)=0; velCurr(1)=0; velCurr(2)=0;

  checkArrival(poseCurr);

//  errIntegral(0)=dt_default*(next_wpt(0)-poseCurr(0));
//  errIntegral(1)=dt_default*(next_wpt(1)-poseCurr(1));
//  errIntegral(2)=dt_default*(next_wpt(2)-poseCurr(2));

//  //Integrator saturation
//  saturationF(errIntegral(0),eImax(0));
//  saturationF(errIntegral(1),eImax(1));
//  saturationF(errIntegral(2),eImax(2));

//  //create PID command
//  uPID(0)=kp(0)*(next_wpt(0)-poseCurr(0)) + kd(0)*(0-velCurr(0)) + ki(0)*errIntegral(0);
//  uPID(1)=kp(1)*(next_wpt(1)-poseCurr(1)) + kd(1)*(0-velCurr(1)) + ki(1)*errIntegral(1);
//  uPID(2)=kp(2)*(next_wpt(2)-poseCurr(2)) + kd(2)*(0-velCurr(2)) + ki(2)*errIntegral(2);

//  limitAcceleration(velCurr,uPID);

  //uses the PVA message from px4_control package
  px4_control::PVA PVA_Ref_msg;
  PVA_Ref_msg.yaw=0.0;  //can try nonzero if optimal

//  //Move to next point with PID
//  PVA_Ref_msg.Pos.x=poseCurr(0)+dt_default*velCurr(0)+dt_default*dt_default*0.5*uPID(0);
//  PVA_Ref_msg.Pos.y=poseCurr(1)+dt_default*velCurr(1)+dt_default*dt_default*0.5*uPID(1);
//  PVA_Ref_msg.Pos.z=poseCurr(2)+dt_default*velCurr(2)+dt_default*dt_default*0.5*uPID(2);
//  PVA_Ref_msg.Vel.x=velCurr(0)+dt_default*uPID(0);
//  PVA_Ref_msg.Vel.y=velCurr(1)+dt_default*uPID(1);
//  PVA_Ref_msg.Vel.z=velCurr(2)+dt_default*uPID(2);
//  PVA_Ref_msg.Acc.x=uPID(0);
//  PVA_Ref_msg.Acc.y=uPID(1);
//  PVA_Ref_msg.Acc.z=uPID(2);

//  //Move to next wpt in one step
//  PVA_Ref_msg.Pos.x=next_wpt(0);
//  PVA_Ref_msg.Pos.y=next_wpt(1);
//  PVA_Ref_msg.Pos.z=next_wpt(2);
//  PVA_Ref_msg.Vel.x=0;
//  PVA_Ref_msg.Vel.y=0;
//  PVA_Ref_msg.Vel.z=0;
//  PVA_Ref_msg.Acc.x=0;
//  PVA_Ref_msg.Acc.y=0;
//  PVA_Ref_msg.Acc.z=0;

  //Move to next wpt in multiple substeps
  //Check to see if the window needs to increase in size
  double dt_x = (next_wpt(0)-poseCurr(0))/vmax(0);
  double dt_y = (next_wpt(1)-poseCurr(1))/vmax(1);
  double dt_z = (next_wpt(2)-poseCurr(2))/vmax(2);
  double estStepsMaxVel = ceil(std::max(dt_x,std::max(dt_y,dt_z)) / dt_default);
  if(estStepsMaxVel>stepsToNextWpt-stepCounter) stepsToNextWpt++;
  //get substep size
  stepCounter++;
  double substep=1-stepCounter*(1.0/stepsToNextWpt);

  //fill message fields
  //The proper way to do this is with discrete integration but handling it with substeps is close enough
  PVA_Ref_msg.Pos.x=next_wpt(0)-substep*(next_wpt(0)-old_pose(0));
  PVA_Ref_msg.Pos.y=next_wpt(1)-substep*(next_wpt(1)-old_pose(1));
  PVA_Ref_msg.Pos.z=next_wpt(2)-substep*(next_wpt(2)-old_pose(2));
  PVA_Ref_msg.Vel.x=next_vel(0)-substep*(next_vel(0)-old_vel(0));
  PVA_Ref_msg.Vel.y=next_vel(1)-substep*(next_vel(1)-old_vel(1));
  PVA_Ref_msg.Vel.z=next_vel(2)-substep*(next_vel(2)-old_vel(2));
  PVA_Ref_msg.Acc.x=next_acc(0)-substep*(next_acc(0)-old_acc(0));
  PVA_Ref_msg.Acc.y=next_acc(1)-substep*(next_acc(1)-old_acc(1));
  PVA_Ref_msg.Acc.z=next_acc(2)-substep*(next_acc(2)-old_acc(2));

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
    old_pose=poseCurr;
    old_vel=velCurr;
    old_acc(0)=0;
    old_acc(1)=0;
    old_acc(2)=0;

    //reset PID params
    errIntegral(0)=0.0;
    errIntegral(1)=0.0;
    errIntegral(2)=0.0;

    //get next waypoint PVA from list
    next_wpt(0) = global_path_msg->pva[0].pos.position.x;
    next_wpt(1) = global_path_msg->pva[0].pos.position.y;
    next_wpt(2) = global_path_msg->pva[0].pos.position.z;
    next_vel(0) = global_path_msg->pva[0].vel.linear.x;
    next_vel(1) = global_path_msg->pva[0].vel.linear.y;
    next_vel(2) = global_path_msg->pva[0].vel.linear.z;
    next_acc(0) = global_path_msg->pva[0].acc.linear.x;
    next_acc(1) = global_path_msg->pva[0].acc.linear.y;
    next_acc(2) = global_path_msg->pva[0].acc.linear.z;

    //estimated steps to next wpt
    double dt_x = (next_wpt(0)-poseCurr(0))/vmax(0);
    double dt_y = (next_wpt(1)-poseCurr(1))/vmax(1);
    double dt_z = (next_wpt(2)-poseCurr(2))/vmax(2);
    stepsToNextWpt = ceil(std::max(dt_x,std::max(dt_y,dt_z)) / dt_default);
    ROS_INFO("Moving to waypoint %f %f %f",next_wpt(0),next_wpt(1),next_wpt(2));
  }else
  {
    next_wpt=poseCurr;
  }
}


//if a single wpt is received
void waypointControl::wptCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  double tempTime=(msg->header.stamp).toSec();
  if(tempTime>wptTime)  //if an update arrives late, ignore it
  {
    wptTime=tempTime;
    //read in next wpt and offset by distance between origin and center
    next_wpt(0)=msg->pose.position.x+arenaCenter(0);
    next_wpt(1)=msg->pose.position.y+arenaCenter(1);
    next_wpt(2)=msg->pose.position.z+arenaCenter(2);
    next_vel.setZero();
    next_acc.setZero();
    old_pose=poseCurr;
    old_vel=velCurr;
    old_acc.setZero();
    errIntegral(0)=0.0;
    errIntegral(1)=0.0;
    errIntegral(2)=0.0;
  }
}


double waypointControl::saturationF(double& xval, const double satbound)
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
//  ROS_INFO("distcurr = %f",thisDist-hitDist);
  if(thisDist<hitDist)
  {
//    t0=time0;
    if(waypointCounter<wptListLen-1)
    {
      waypointCounter++;
      //update new and old wpt locations
      old_pose=next_wpt;
      next_wpt(0)=global_path_msg->pva[waypointCounter].pos.position.x+arenaCenter(0);
      next_wpt(1)=global_path_msg->pva[waypointCounter].pos.position.y+arenaCenter(1);
      next_wpt(2)=global_path_msg->pva[waypointCounter].pos.position.z+arenaCenter(2);

      old_vel=next_vel;
      next_vel(0)=global_path_msg->pva[waypointCounter].vel.linear.x;
      next_vel(1)=global_path_msg->pva[waypointCounter].vel.linear.y;
      next_vel(2)=global_path_msg->pva[waypointCounter].vel.linear.z;

      old_acc=next_acc;
      next_acc(0)=global_path_msg->pva[waypointCounter].acc.linear.x;
      next_acc(1)=global_path_msg->pva[waypointCounter].acc.linear.y;
      next_acc(2)=global_path_msg->pva[waypointCounter].acc.linear.z;

      //if you have the next time, find segment chunks that correspond to it. Otherwise, guesstimate.
      if(global_path_msg->pva[waypointCounter].header.stamp.toSec() > 1e-4 )
      {
        dtNextWpt=(global_path_msg->pva[waypointCounter].header.stamp).toSec() -
                   (global_path_msg->pva[waypointCounter-1].header.stamp).toSec();      
      }else
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
    }else
    {
      ROS_INFO("Final waypoint reached, holding station.");

      //Set old VA to 0 to hold station
      next_vel.setZero();
      next_acc.setZero();
      old_vel.setZero();
      old_acc.setZero();
    }
  }
}

}//end namespace


