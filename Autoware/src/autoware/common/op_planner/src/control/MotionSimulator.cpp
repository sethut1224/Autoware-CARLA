/// \file MotionSimulator.cpp
/// \brief OpenPlanner's Motion Simulator
/// \author Hatem Darweesh
/// \date Jan 16, 2021

#include "op_planner/control/MotionSimulator.h"

namespace PlannerHNS
{

MotionSimulator::MotionSimulator()
{
	m_CurrentVelocity =  m_CurrentVelocityD = 0;
	m_CurrentSteering = m_CurrentSteeringD = 0;
	m_CurrentShift =  m_CurrentShiftD = SHIFT_POS_NN;
	m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;
	m_WheelBase = 2.7;
	m_MaxWheelAngle = 0.42;
	m_SteeringDelay = 0;
}

MotionSimulator::~MotionSimulator()
{
}

WayPoint MotionSimulator::getStatePose()
{
	return m_State;
}

void MotionSimulator::UpdateStateHeight(const double& new_height)
{
	m_State.pos.z = new_height;
}

double MotionSimulator::getStateVelocity()
{
	return m_CurrentVelocity;
}

double MotionSimulator::getStateSteering()
{
	return m_CurrentSteering;
}

void MotionSimulator::Init(const double& wheel_base, const double& max_wheel_angle, const double& steering_delay)
{
	m_CurrentVelocity = m_CurrentVelocityD = 0;
	m_CurrentSteering = m_CurrentSteeringD = 0;
	m_CurrentShift = m_CurrentShiftD = SHIFT_POS_NN;
	m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;
	/**
	 * TODO If wheel base length is zero, differential drive model should be used instead of the bicycle model
	 */
	m_WheelBase = wheel_base;
	m_MaxWheelAngle = max_wheel_angle;
	m_SteeringDelay = steering_delay;
}

 void MotionSimulator::FirstLocalizeMe(const WayPoint& initCarPos)
 {
	m_State = initCarPos;
 }

void MotionSimulator::SetSimulatedTargetOdometryReadings(const double& velocity_d, const double& steering_d, const SHIFT_POS& shift_d)
{
	m_CurrentVelocityD = velocity_d;
	m_CurrentSteeringD = steering_d;
	m_CurrentShiftD = shift_d;
}

void MotionSimulator::LocalizeMe(const double& dt)
{
	if(m_CurrentShift == SHIFT_POS_DD)
	{
		m_State.pos.a += m_CurrentVelocity * dt * tan(m_CurrentSteering)  / m_WheelBase;
		m_State.pos.a = UtilityHNS::UtilityH::FixNegativeAngle(atan2(sin(m_State.pos.a), cos(m_State.pos.a)));
		m_State.pos.x += m_CurrentVelocity * dt * cos(m_State.pos.a);
		m_State.pos.y += m_CurrentVelocity * dt * sin(m_State.pos.a);

	}
	else if(m_CurrentShift == SHIFT_POS_RR )
	{
		m_State.pos.a -= m_CurrentVelocity * dt * tan(m_CurrentSteering)  / m_WheelBase;
		m_State.pos.a = UtilityHNS::UtilityH::FixNegativeAngle(atan2(sin(m_State.pos.a), cos(m_State.pos.a)));
		m_State.pos.x -= m_CurrentVelocity * dt * cos(m_State.pos.a);
		m_State.pos.y -= m_CurrentVelocity * dt * sin(m_State.pos.a);
	}
}

 void MotionSimulator::UpdateState(const double& dt, const PlannerHNS::VehicleState& state)
  {
	 if(m_SteeringDelay == 0)
	 {
		 m_CurrentSteering = m_CurrentSteeringD;
	 }
	 else
	 {
		 double curr_steering = m_CurrentSteering;
		 double steering_diff = m_CurrentSteeringD - curr_steering;
		 double max_dt_steer = ( m_MaxWheelAngle * dt ) / (m_SteeringDelay*2.0);
		 double diffSign = UtilityHNS::UtilityH::GetSign(steering_diff);

		 if(fabs(steering_diff) > max_dt_steer)
		 {
			 curr_steering += diffSign * max_dt_steer;
		 }
		 else
		 {
			 curr_steering += steering_diff;
		 }

		 m_CurrentSteering = curr_steering;
	 }

	 m_CurrentShift = m_CurrentShiftD;
	 m_CurrentVelocity = m_CurrentVelocityD;
  }

 void MotionSimulator::SimulateOdoPosition(const double& dt, const PlannerHNS::VehicleState& vehicleState)
 {
	SetSimulatedTargetOdometryReadings(vehicleState.speed, vehicleState.steer, vehicleState.shift);
	UpdateState(dt, vehicleState);
	LocalizeMe(dt);
 }

} /* namespace PlannerHNS */
