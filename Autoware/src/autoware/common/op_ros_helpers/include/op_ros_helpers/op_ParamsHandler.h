/// \file  op_ParamsHandler.h
/// \brief Handle OpenPlanner dynamic parameters messages
/// \author Hatem Darweesh
/// \date October 4, 2021

#ifndef OPPARAMSHANDLER_H_
#define OPPARAMSHANDLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlannerCommonDef.h"


namespace PlannerHNS
{

class ParamsHandler
{
private:

	PlannerHNS::PlanningParams* m_pPlanningParams = nullptr;
	PlannerHNS::CAR_BASIC_INFO* m_pCarInfo = nullptr;
	PlannerHNS::ControllerParams* m_pControlParams = nullptr;

	ros::Subscriber sub_rollouts_number;
	ros::Publisher pub_rollouts_number;
	void callbackGetRolloutsNumber(const std_msgs::Int32& msg);

	ros::Subscriber sub_rollouts_density;
	ros::Publisher pub_rollouts_density;
	void callbackGetRolloutsDensity(const std_msgs::Float64& msg);

public:
	void InitHandler(ros::NodeHandle& nh, PlannerHNS::PlanningParams* pParams, PlannerHNS::CAR_BASIC_INFO* pCarInfo, PlannerHNS::ControllerParams* pControlParams);

	/**
	 * will return false if the value is out of range and can't be used
	 */
	bool UpdateRolloutsNumber(const int rollouts_number);

	/**
	 * will return false if the value is out of range and can't be used
	 */
	bool UpdateRolloutsDensity(const double rollouts_number);
};

} /* namespace PlannerHNS */

#endif /* OPPARAMSHANDLER_H_ */
