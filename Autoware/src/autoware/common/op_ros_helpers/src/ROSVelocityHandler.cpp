/// \file  ROSVelocityHandler.h
/// \brief Handle velocity update functionality for OpenPlanner, such as message odometry, vehicle status, CAN
/// \author Hatem Darweesh
/// \date June 19, 2021

#include "op_ros_helpers/ROSVelocityHandler.h"
#include "op_ros_helpers/op_ROSHelpers.h"

namespace PlannerHNS
{

VelocityHandler::VelocityHandler()
{
	m_pVehicleState = nullptr;
	m_pCurrentPose = nullptr;
}

VelocityHandler::~VelocityHandler()
{

}

void VelocityHandler::InitVelocityHandler(ros::NodeHandle& nh, const PlannerHNS::CAR_BASIC_INFO& car_info, PlannerHNS::VehicleState* vehicle_status, PlannerHNS::WayPoint* curr_pose)
{
	m_pVehicleState = vehicle_status;
	m_pCurrentPose = curr_pose;
	m_CarInfo = car_info;

	int iVelSource = 0;
	nh.getParam("/op_common_params/velocitySource", iVelSource);
	std::string velocity_topic;

	if(iVelSource == 0)
	{
		nh.getParam("/op_common_params/vel_odom_topic", velocity_topic);
		sub_robot_odom = nh.subscribe(velocity_topic, 1, &VelocityHandler::callbackGetRobotOdom, this);
	}
	else if(iVelSource == 1)
	{
		nh.getParam("/op_common_params/vel_curr_topic", velocity_topic);
		sub_current_velocity = nh.subscribe(velocity_topic, 1, &VelocityHandler::callbackGetAutowareStatus, this);
	}
	else if(iVelSource == 2)
	{
		nh.getParam("/op_common_params/vel_can_topic", velocity_topic);
		sub_can_info = nh.subscribe(velocity_topic, 1, &VelocityHandler::callbackGetCANInfo, this);
	}
	else if(iVelSource == 3)
	{
		nh.getParam("/op_common_params/vehicle_status_topic", velocity_topic);
		sub_vehicle_status = nh.subscribe(velocity_topic, 1, &VelocityHandler::callbackGetVehicleStatus, this);
	}
}

void VelocityHandler::callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	if(m_pVehicleState == nullptr || m_pCurrentPose == nullptr) return;

	m_pVehicleState->speed = msg->twist.linear.x;
	m_pCurrentPose->v = m_pVehicleState->speed;

	if(fabs(m_pCurrentPose->v) > 0.1)
	{
		m_pVehicleState->steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/m_pCurrentPose->v);
	}
}

void VelocityHandler::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	if(m_pVehicleState == nullptr || m_pCurrentPose == nullptr) return;

	m_pVehicleState->speed = msg->speed/3.6;
	m_pCurrentPose->v = m_pVehicleState->speed;
	m_pVehicleState->steer = msg->angle * m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
}

void VelocityHandler::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	if(m_pVehicleState == nullptr || m_pCurrentPose == nullptr) return;

	m_pVehicleState->speed = msg->twist.twist.linear.x;
	m_pCurrentPose->v = m_pVehicleState->speed ;
	if(msg->twist.twist.linear.x != 0)
	{
		m_pVehicleState->steer = atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	}
}

void VelocityHandler::callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg)
{
	if(m_pVehicleState == nullptr || m_pCurrentPose == nullptr) return;

	m_pVehicleState->speed = msg->speed/3.6;
	m_pVehicleState->steer = -msg->angle*DEG2RAD;
	m_pCurrentPose->v = m_pVehicleState->speed;
}

} /* namespace PlannerHNS */
