

// These are pieces and chunks of code that are currently unused but may be reused

/*
void waypointControl::wptListCallback(const nav_msgs::Path::ConstPtr &msg)
{
  //int nn= ;//length of pose list
  numPathsSoFar++;
  if(msg) {
    wptListLen=msg->poses.size();
    global_path_msg = msg;


    waypointCounter=0;
    counter++; //totla number of waypoint list/paths received

    errIntegral(0)=0.0;
    errIntegral(1)=0.0;
    errIntegral(2)=0.0;
    next_wpt(0) = global_path_msg->pva[0].pos.position.x;
    next_wpt(1) = global_path_msg->pva[0].pos.position.y;
    next_wpt(2) = global_path_msg->pva[0].pos.position.z;
    next_vel(0) = global_path_msg->pva[0].vel.li These are pieces and chunks of |near.x;
    next_vel(1) = global_path_msg->pva[0].vel.linear.y;
    next_vel(2) = global_path_msg->pva[0].vel.linear.z;
    next_acc(0) = global_path_msg->pva[0].acc.linear.x;
    next_acc(1) = global_path_msg->pva[0].acc.linear.y;
    next_acc(2) = global_path_msg->pva[0].acc.linear.z
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




    //testing reference outputs
    int k=global_path_msg->pva.size();
    for(int i=1; i<k; i++)
    {
      Eigen::Vector3d vold, vnew;
      vold=Eigen::Vector3d(
        global_path_msg->pva[i-1].pos.position.x,
          global_path_msg->pva[i-1].pos.position.y,
          global_path_msg->pva[i-1].pos.position.z);
      vnew=Eigen::Vector3d(
        global_path_msg->pva[i].pos.position.x,
          global_path_msg->pva[i].pos.position.y,
          global_path_msg->pva[i].pos.position.z);
      ROS_INFO("dtt2=%f   dh=%f" , (global_path_msg->pva[i].header.stamp).toSec() -
                   (global_path_msg->pva[i-1].header.stamp).toSec(),
                   (vnew-vold).norm());
    }



*/


