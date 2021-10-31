/// \file  op_ParamsHandler.h
/// \brief Handle OpenPlanner dynamic parameters messages
/// \author Hatem Darweesh
/// \date October 4, 2021

#include "op_ros_helpers/op_ParamsHandler.h"
#include "op_ros_helpers/op_ROSHelpers.h"

namespace PlannerHNS
{

void ParamsHandler::InitHandler(ros::NodeHandle& nh, PlannerHNS::PlanningParams* pParams, PlannerHNS::CAR_BASIC_INFO* pCarInfo, PlannerHNS::ControllerParams* pControlParams)
{
	m_pPlanningParams = pParams;
	m_pCarInfo = pCarInfo;
	m_pControlParams = pControlParams;

	pub_rollouts_number = nh.advertise<std_msgs::Int32>("op_update_rollouts_number", 1);
	sub_rollouts_number = nh.subscribe("op_update_rollouts_number", 1, &ParamsHandler::callbackGetRolloutsNumber, this);

	pub_rollouts_density = nh.advertise<std_msgs::Float64>("op_update_rollouts_density", 1);
	sub_rollouts_density = nh.subscribe("op_update_rollouts_density", 1, &ParamsHandler::callbackGetRolloutsDensity, this);
}

bool ParamsHandler::UpdateRolloutsNumber(const int rollouts_number)
{
	if(rollouts_number >= 0 && rollouts_number <= 1000)
	{
		std_msgs::Int32 msg;
		msg.data = rollouts_number;
		pub_rollouts_number.publish(msg);
		return true;
	}

	return false;
}

bool ParamsHandler::UpdateRolloutsDensity(const double rollouts_density)
{
	if(rollouts_density >= 0.0 && rollouts_density <= 20.0)
	{
		std_msgs::Float64 msg;
		msg.data = rollouts_density;
		pub_rollouts_density.publish(msg);
		return true;
	}

	return false;
}

void ParamsHandler::callbackGetRolloutsNumber(const std_msgs::Int32& msg)
{
	if(m_pPlanningParams != nullptr && msg.data >= 0 && msg.data <= 1000)
	{
		m_pPlanningParams->rollOutNumber = msg.data;
	}
}

void ParamsHandler::callbackGetRolloutsDensity(const std_msgs::Float64& msg)
{
	if(m_pPlanningParams != nullptr && msg.data > 0.0 && msg.data <= 20.0)
	{
		m_pPlanningParams->rollOutDensity = msg.data;
	}
}

} /* namespace PlannerHNS */
