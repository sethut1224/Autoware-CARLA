
/// \file MotionSimulator.h
/// \brief OpenPlanner's Motion Simulator
/// \author Hatem Darweesh
/// \date Jan 16, 2021


#ifndef MOTIONSIMULATOR_H_
#define MOTIONSIMULATOR_H_

#include "op_planner/PlannerCommonDef.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlanningHelpers.h"

namespace PlannerHNS
{

class MotionSimulator
{
private:
	WayPoint m_State;
	double m_WheelBase;
	double m_MaxWheelAngle;
	double m_SteeringDelay;

	double m_CurrentVelocity, m_CurrentVelocityD; //meter/second
	double m_CurrentSteering, m_CurrentSteeringD; //radians
	SHIFT_POS m_CurrentShift , m_CurrentShiftD;

	double m_CurrentAccSteerAngle; //degrees steer wheel range
	double m_CurrentAccVelocity; // kilometer/hour

	void SetSimulatedTargetOdometryReadings(const double& velocity_d, const double& steering_d, const SHIFT_POS& shift_d);
	void LocalizeMe(const double& dt); // in seconds
	void UpdateState(const double& dt, const VehicleState& state);

public:

	MotionSimulator();
	virtual ~MotionSimulator();
	void Init(const double& wheel_base, const double& max_wheel_angle, const double& steering_delay);
	void FirstLocalizeMe(const WayPoint& initCarPos);
	void SimulateOdoPosition(const double& dt, const VehicleState& vehicleState);
	WayPoint getStatePose();
	double getStateVelocity();
	double getStateSteering();
	void UpdateStateHeight(const double& new_height);
};

} /* namespace PlannerHNS */

#endif /* MOTIONSIMULATOR_H_ */
