/// \file  ROSVelocityHandler.h
/// \brief Handle velocity update functionality for OpenPlanner, such as message odometry, vehicle status, CAN
/// \author Hatem Darweesh
/// \date June 19, 2021

#ifndef ROSVELHANDLER_H_
#define ROSVELHANDLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include "autoware_msgs/VehicleStatus.h"
#include "autoware_can_msgs/CANInfo.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlannerCommonDef.h"


namespace PlannerHNS
{

class VelocityHandler
{
private:

	PlannerHNS::VehicleState* m_pVehicleState;
	PlannerHNS::WayPoint* m_pCurrentPose;
	PlannerHNS::CAR_BASIC_INFO m_CarInfo;

	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_vehicle_status;

	void callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg);

public:
	VelocityHandler();
	virtual ~VelocityHandler();
	void InitVelocityHandler(ros::NodeHandle& nh, const PlannerHNS::CAR_BASIC_INFO& car_info, PlannerHNS::VehicleState* vehicle_status, PlannerHNS::WayPoint* curr_pose);
};

} /* namespace PlannerHNS */

#endif /* ROSVELHANDLER_H_ */
