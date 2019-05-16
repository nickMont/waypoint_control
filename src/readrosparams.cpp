#include "waypointcontrol.hpp"


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

	ros::param::get("waypoint_control_node/takeoffHeight", takeoffHeight_);

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
