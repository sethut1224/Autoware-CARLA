
/// \file op_controller.cpp
/// \brief PID based trajectory follower and velocity controller
/// \author Hatem Darweesh
/// \date July 04, 2020

#include "op_planner/control/op_controller.h"
#include "op_planner/PlanningHelpers.h"
#include <cmath>
#include <stdlib.h>
#include <iostream>

namespace PlannerHNS
{

MotionControl::MotionControl()
{
	m_TargetAngle = 0;
	m_TargetSpeed = 0;
	m_PrevAngleError = 0;
	m_PrevSpeedError = 0;
	m_PrevSpeed = 0;
	m_iNextTest = 0;
	m_FollowingDistance = 0;
	m_LateralError 		= 0;
	m_PrevDesiredTorque	= 0;
	m_PrevDesiredSteerAngle = 0;
	m_iPrevWayPoint = -1;
	m_AccelerationSum = 0;
	m_nAccelerations = 0;
	m_AverageAcceleration = 0;
	m_PrevFollowDistance = 0;
	m_AverageRelativeSpeed = 0;
	m_TargetAcceleration = 0;
	m_DesiredDistance = 0;
	m_DesiredSafeDistance = 0;
	m_PrevDistanceError = 0;
	m_ffEstimatedVelocity = 0;
	m_PredictedVelMinusRealVel = 0;

	UtilityHNS::UtilityH::GetTickCount(m_SteerDelayTimer);
	UtilityHNS::UtilityH::GetTickCount(m_VelocityDelayTimer);
	ResetLogTime(0,0);
}

void MotionControl::Init(const ControllerParams& params, const ControllerHyperParams& hyper_params, const CAR_BASIC_INFO& vehicleInfo)
{
	m_HyperParams = hyper_params;

	if(m_HyperParams.bEnableCalibration)
	{
		InitCalibration();
	}

	m_Params = params;
	m_VehicleInfo = vehicleInfo;

	m_pidSteering.Init(m_Params.Steering_Gain.kP, m_Params.Steering_Gain.kI, m_Params.Steering_Gain.kD);
	m_pidSteering.Setlimit(m_VehicleInfo.max_wheel_angle, -m_VehicleInfo.max_wheel_angle);

	m_pidSteerWheelTorque.Init(m_Params.Torque_Gain.kP, m_Params.Torque_Gain.kI, m_Params.Torque_Gain.kD);
	m_pidSteerWheelTorque.Setlimit(m_VehicleInfo.max_steer_torque, m_VehicleInfo.min_steer_torque);

	m_pidAccelPedal.Init(m_Params.Accel_Gain.kP, m_Params.Accel_Gain.kI, m_Params.Accel_Gain.kD);
	m_pidAccelPedal.Setlimit(m_VehicleInfo.max_accel_value, 0);

	m_pidFollow.Init(m_Params.Accel_Gain.kP*m_HyperParams.follow_velocity_gain_ratio,
			m_Params.Accel_Gain.kI*m_HyperParams.follow_velocity_gain_ratio,
			m_Params.Accel_Gain.kD*m_HyperParams.follow_velocity_gain_ratio);
	m_pidFollow.Setlimit(m_VehicleInfo.max_accel_value, 0);

	m_pidBrakePedal.Init(m_Params.Brake_Gain.kP, m_Params.Brake_Gain.kI, m_Params.Brake_Gain.kD);
	m_pidBrakePedal.Setlimit(m_VehicleInfo.max_brake_value, 0);
}

MotionControl::~MotionControl()
{
	if(m_HyperParams.bEnableLogs)
	{
		std::ostringstream fileName;
		if(m_HyperParams.log_file_path.size() == 0)
			fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ControlLogFolderName;
		else
			fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_HyperParams.log_file_path + UtilityHNS::DataRW::ControlLogFolderName;

		UtilityHNS::DataRW::WriteLogData(fileName.str(), "ControlLog",
				"dt,t,X,Y,"
				"heading,target_angle,angle_err,torque,"
				"state,stop_distance,acceleration,avg_relative_speed,pred_relative_speed,"
				"speed,target_speed,target_accel,speed_err,distance_err,target_distance,safe_distance,accel_stroke,"
				"brake_stroke,lateral_error,iIndex,pathSize,avg_accel,total_accel,feed_forward_vel,ff_minus_speed",
				m_LogData);

		if(m_HyperParams.bEnableCalibration)
		{
			UtilityHNS::DataRW::WriteLogData(fileName.str(), "SteeringCalibrationLog",
					"time, reset, start A, end A, desired A, dt, vel", m_SteerCalibrationData);

			UtilityHNS::DataRW::WriteLogData(fileName.str(), "VelocityCalibrationLog",
					"time, reset, start V, end V, desired V, dt, steering", m_VelocityCalibrationData);
		}

		if(m_HyperParams.bEnableSteeringMode)
		{
			UtilityHNS::DataRW::WriteLogData(fileName.str(), "SteeringPIDLog",m_pidSteering.ToStringHeader(), m_LogSteerPIDData );
		}
		else
		{
			UtilityHNS::DataRW::WriteLogData(fileName.str(), "SteeringTorquePIDLog",m_pidSteerWheelTorque.ToStringHeader(), m_LogTorquePIDData );
		}

		if(m_HyperParams.bEnableVelocityMode)
		{
			//UtilityHNS::DataRW::WriteLogData(fileName.str(), "VelocityPIDLog",m_pidVelocity.ToStringHeader(), m_LogVelocityPIDData );
		}
		else
		{
			UtilityHNS::DataRW::WriteLogData(fileName.str(), "AccelPIDLog",m_pidAccelPedal.ToStringHeader(), m_LogAccelerationPIDData );
			UtilityHNS::DataRW::WriteLogData(fileName.str(), "BrakePIDLog",m_pidBrakePedal.ToStringHeader(), m_LogBrakingPIDData );
		}

		UtilityHNS::DataRW::WriteLogData(fileName.str(), "FollowPIDLog",m_pidFollow.ToStringHeader(), m_LogFollowPIDData );
	}
}

void MotionControl::ResetLogTime(const double& v0, const double& v1)
{
	double time_total = UtilityHNS::UtilityH::GetTimeDiffNow(m_LogTimer);
	if(time_total != 0)
	{
		m_TotalAcceleration = (v1 - v0) / time_total;
	}

	if(m_nAccelerations != 0)
	{
		m_AverageAcceleration = m_AccelerationSum / m_nAccelerations;
	}

	UtilityHNS::UtilityH::GetTickCount(m_LogTimer);
}

void MotionControl::UpdateCurrentPath(const std::vector<PlannerHNS::WayPoint>& path)
{
	m_Path = path;
}

bool MotionControl::FindNextWayPoint(const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
		const double& velocity, PlannerHNS::WayPoint& pursuite_point, PlannerHNS::WayPoint& prep,
		double& lateral_err, double& follow_distance)
{
	if(path.size()==0) return false;

	follow_distance = PlanningHelpers::CalculateLookAheadDistance(m_Params.SteeringDelay, velocity, m_Params.minPursuiteDistance);
	//follow_distance = m_Params.minPursuiteDistance;

	RelativeInfo info;
	PlanningHelpers::GetRelativeInfo(path, state, info);
	unsigned int dummy_index = 0;
	pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(path, info, follow_distance, dummy_index);
	prep = info.perp_point;
	lateral_err = info.perp_distance;
	m_iPrevWayPoint = info.iFront;

	return true;
}

void MotionControl::SteerControllerUpdate(const double& dt, const PlannerHNS::WayPoint& CurrPose, const PlannerHNS::WayPoint& TargetPose,
		const PlannerHNS::VehicleState& CurrStatus, const PlannerHNS::BehaviorState& CurrBehavior,
		const double& lateralErr, double& desiredSteerAngle)
{
	if(CurrBehavior.state != INITIAL_STATE && CurrBehavior.state != FINISH_STATE)
	{
		AngleControllerPart(dt, CurrPose, TargetPose, lateralErr, desiredSteerAngle);

		if(desiredSteerAngle > m_VehicleInfo.max_wheel_angle)
		{
			desiredSteerAngle = m_VehicleInfo.max_wheel_angle;
		}
		else if(desiredSteerAngle < -m_VehicleInfo.max_wheel_angle)
		{
			desiredSteerAngle = -m_VehicleInfo.max_wheel_angle;
		}
	}
	else
	{
		desiredSteerAngle = 0;
	}
}

void MotionControl::TorqueControllerUpdate(const double& dt, const PlannerHNS::WayPoint& CurrPose, const PlannerHNS::WayPoint& TargetPose,
		const PlannerHNS::VehicleState& CurrStatus, const PlannerHNS::BehaviorState& CurrBehavior,
		const double& lateralErr, double& desiredSteerTorque)
{
	if(CurrBehavior.state != INITIAL_STATE && CurrBehavior.state != FINISH_STATE)
	{
		TorqueControllerPart(dt, CurrPose, TargetPose, lateralErr, desiredSteerTorque);
	}
	else
	{
		desiredSteerTorque = 0;
	}
}

void MotionControl::TorqueControllerPart(const double& dt, const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
		const double& lateral_error, double& torque_d)
{
	double current_a = UtilityHNS::UtilityH::SplitPositiveAngle(state.pos.a);
	double target_a = atan2(way_point.pos.y - state.pos.y, way_point.pos.x - state.pos.x);
	double e =  UtilityHNS::UtilityH::SplitPositiveAngle(target_a - current_a);

	if((e > 0 && m_PrevAngleError < 0) || (e < 0 && m_PrevAngleError > 0))
	{
		m_pidSteerWheelTorque.ResetI();
	}

	//ToDo Activate this later, check first if this condition ever happens
//	if(e > M_PI_2 || e < -M_PI_2)
//	{
//		torque_d = m_PrevDesiredTorque;
//		return -1;
//	}

	//TODO use lateral error instead of angle error
	torque_d = m_pidSteerWheelTorque.getTimeDependentPID(e, dt);

	m_TargetAngle = target_a;
	m_PrevAngleError = e;
	m_PrevDesiredTorque = torque_d;
}

void MotionControl::AngleControllerPart(const double& dt, const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
		const double& lateral_error, double& steer_angle_d)
{
	double current_a = UtilityHNS::UtilityH::SplitPositiveAngle(state.pos.a);
	double target_a = atan2(way_point.pos.y - state.pos.y, way_point.pos.x - state.pos.x);
	//Use angle Error
	double e =  UtilityHNS::UtilityH::SplitPositiveAngle(target_a - current_a);

	//Use CTE (cross track error)
	//TODO use lateral error instead of angle error
	//RelativeInfo info;
	//PlanningHelpers::GetRelativeInfo(m_Path, state, info);
	//double e = info.perp_distance;

	if((e > 0 && m_PrevAngleError < 0) || (e < 0 && m_PrevAngleError > 0))
	{
		m_pidSteering.ResetI();
	}

	steer_angle_d = m_pidSteering.getTimeDependentPID(e, dt);

	m_TargetAngle = target_a;
	m_PrevAngleError = e;
	m_PrevDesiredSteerAngle = steer_angle_d;
}

double MotionControl::EstimateFutureVelocity(double v0, double v_d, double accel_stroke, double brake_stroke, double time_elapsed)
{
	bool bPedalSwitch = false;
	double init_speed_jump = m_Params.avg_acceleration * (m_Params.accel_init_delay + time_elapsed);

	if((m_PrevDesiredState.accel_stroke > 0 && accel_stroke == 0) ||
			(m_PrevDesiredState.brake_stroke > 0 && brake_stroke == 0) || (m_PrevDesiredState.accel_stroke > 0 && v0 < init_speed_jump && v_d > v0))
	{
		bPedalSwitch = true;
	}

	double accel = 0;
	double delay = time_elapsed;
	if(v_d >= v0)
	{
		accel = m_Params.avg_acceleration;

		if(bPedalSwitch)
			delay += m_Params.accel_init_delay;
		else
			delay += m_Params.accel_avg_delay;
	}
	else if(v_d < v0)
	{
		accel = m_Params.avg_deceleration;

		if(bPedalSwitch)
			delay += m_Params.brake_init_delay;
		else
			delay += m_Params.brake_avg_delay;
	}

	double a_dt = accel * delay;

	//Initial case
	double ff_vel = 0;
	ff_vel = v0 + a_dt;

//	std::cout << "Current Additional Velocity : " << a_dt << ", v_d: " << v_d << ", accel_stroke: " <<  accel_stroke << ", brake_stroke: " << brake_stroke << std::endl;

	return ff_vel;
}

void MotionControl::CalculateVelocityDesired(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& vel_d, double& acc_d, double& distance_d, double& safe_d)
{
	double desired_acceleration = 0, desired_velocity = 0;

	/**
	 * Forward State, No obstacle, just try to follow general max speed profile for the trajectory
	 */
	if(CurrBehavior.state == FORWARD_STATE || CurrBehavior.state == OBSTACLE_AVOIDANCE_STATE )
	{
		if(CurrStatus.speed <= CurrBehavior.maxVelocity) //accelerate with half the max accel
		{
			desired_acceleration = m_VehicleInfo.max_acceleration;
		}
		else //brake with half the max decel
		{
			desired_acceleration = m_VehicleInfo.max_deceleration/2.0;
		}

		desired_velocity = CurrBehavior.maxVelocity;
	}
	/**
	 * Follow State, try to keep safe distance to the object in front, the faster the car in from goinf the larget this safe distance
	 */
	else if(CurrBehavior.state == FOLLOW_STATE) // maintain
	{
		distance_d = CurrBehavior.followDistance;
		safe_d = m_Params.min_safe_follow_distance;

		if(distance_d > safe_d) // maintain distance
		{
			desired_acceleration = m_VehicleInfo.max_acceleration;
		}
		else if(distance_d < safe_d*0.75) // too close , brake hard
		{
			desired_acceleration = m_VehicleInfo.max_deceleration;
		}
		else if(distance_d < safe_d) // use engine brake
		{
			desired_acceleration = m_VehicleInfo.max_deceleration/2.0;
		}

		desired_velocity = CurrBehavior.maxVelocity;

		/**
		 * TODO Add more safe follow distance according to the lead vehicle velocity
		 * safe_d += ~desired_velocity
		 */
	}
	/**
	 * Stop With distance , in these state the vehicle should stop within a designated distance "stopDistance". and should go to complete stop, speed = 0
	 */
	else if(CurrBehavior.state == TRAFFIC_LIGHT_STOP_STATE || CurrBehavior.state == STOP_SIGN_STOP_STATE || CurrBehavior.state == STOPPING_STATE)
	{
		if(CurrBehavior.stopDistance > 0)
		{
			desired_acceleration = (-CurrStatus.speed*CurrStatus.speed)/(2.0*CurrBehavior.stopDistance);
			desired_velocity = CurrStatus.speed + (desired_acceleration * dt);
		}
		else
		{
			desired_velocity = 0;
			desired_acceleration = m_VehicleInfo.max_deceleration;
		}
	}
	/**
	 * No other supported state, vehicle should stop if unknown state is sent
	 */
	else
	{
		desired_velocity = 0;
		desired_acceleration = m_VehicleInfo.max_deceleration;
	}

	/**
	 * Never target velocity higher than the one assigned to OpenPlanner in op_common_params
	 */
	if(desired_velocity > m_VehicleInfo.max_speed_forward)
	{
		desired_velocity = m_VehicleInfo.max_speed_forward;
	}
	else if(desired_velocity < 0)
	{
		std::cout << "Desired velocity shouldn't be Zero !! check op_controller !!!! " << std::endl;
		desired_velocity = 0;
	}

	vel_d = desired_velocity;
	acc_d = desired_acceleration;
}

int MotionControl::StrokeControllerUpdateForOpenPlannerInternalACC(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredAccel, double& desiredBrake, PlannerHNS::SHIFT_POS& desiredShift)
{
	double e_v = 0;
	double desired_velocity = CurrBehavior.maxVelocity;
	e_v = (desired_velocity - CurrStatus.speed); //Target max velocity error

	desiredAccel = m_pidAccelPedal.getTimeDependentPID(e_v, dt);
	if(e_v < 0 && desiredAccel < 0.5)
	{
		desiredBrake = m_pidBrakePedal.getTimeDependentPID(-e_v, dt);
		m_pidAccelPedal.Reset();
	}
	else
	{
		m_pidBrakePedal.Reset();
		desiredBrake = 0;
	}

	desiredShift = PlannerHNS::SHIFT_POS_DD;

	m_TargetSpeed = desired_velocity;
	m_TargetAcceleration = 0;
	m_PrevSpeedError = e_v;
	m_PrevDistanceError = 0;
	m_DesiredDistance = 0;
	m_DesiredSafeDistance = 0;
	m_PredictedVelMinusRealVel = 0;

	return 1;

}

int MotionControl::StrokeControllerUpdateTwoPID(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredAccel, double& desiredBrake, PlannerHNS::SHIFT_POS& desiredShift)
{

	double e_d = 0, e_v = 0;
	double desired_velocity = 0, desired_acceleration = 0, desired_distance = 0, safe_follow_distance = 0;

	CalculateVelocityDesired(dt, CurrStatus, CurrBehavior, desired_velocity, desired_acceleration, desired_distance, safe_follow_distance);

	e_d = (desired_distance - safe_follow_distance); //Follow distance error
	e_v = (desired_velocity - m_ffEstimatedVelocity);


	/**
	 * Reset PID controller of switching from follow state to any other state other vice versa.
	 */
	if(CurrBehavior.state != m_PrevBehaviorStatus.state && (CurrBehavior.state == FOLLOW_STATE || m_PrevBehaviorStatus.state == FOLLOW_STATE))
	{
		m_pidFollow.Reset();
		m_pidAccelPedal.Reset();
		m_pidBrakePedal.Reset();
	}

	double accel_d = 0, accel_v = 0, brake_v = 0;
	if(desired_acceleration >= m_Params.avg_engine_brake_accel) //accelerate , cruise, small decelerate
	{
		m_pidBrakePedal.Reset();
		accel_v = m_pidAccelPedal.getTimeDependentPID(e_v, dt);
		accel_d = m_pidFollow.getTimeDependentPID(e_d, dt);
		brake_v = 0;
	}
	else  // braking
	{
		m_pidAccelPedal.Reset();
		m_pidFollow.Reset();
		if(CurrBehavior.state == FOLLOW_STATE)
		{
			brake_v = m_pidBrakePedal.getTimeDependentPID(-e_d, dt);
		}
		else
		{
			brake_v = m_pidBrakePedal.getTimeDependentPID(-e_v, dt);
		}
		desiredAccel = 0;
	}

	/**
	 * In case of follow state, use the minimum from follow PID and Accel PID. to be able to not exceed the max target velocity
	 */
	if(CurrBehavior.state == FOLLOW_STATE)
	{
		if(accel_d < accel_v)
		{
			desiredAccel = accel_d;
		}
		else
		{
			desiredAccel = accel_v;
		}
	}
	else
	{
		desiredAccel = accel_v;
	}

	desiredBrake = brake_v;
	desiredShift = PlannerHNS::SHIFT_POS_DD;

	m_TargetSpeed = desired_velocity;
	m_TargetAcceleration = desired_acceleration;
	m_PrevSpeedError = e_v;
	m_PrevDistanceError = e_d;
	m_DesiredDistance = desired_distance;
	m_DesiredSafeDistance = safe_follow_distance;
	m_PredictedVelMinusRealVel = m_ffEstimatedVelocity - CurrStatus.speed;

	return 1;
}

void MotionControl::VelocityControllerUpdateUsingACC(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVelocity, PlannerHNS::SHIFT_POS& desiredShift)
{
	desiredVelocity = PlanningHelpers::GetACCVelocityModelBased(dt, CurrStatus.speed, m_VehicleInfo, m_Params, CurrBehavior);
	desiredShift = PlannerHNS::SHIFT_POS_DD;

	//desiredVelocity = m_VehicleInfo.max_speed_forward;

	m_TargetSpeed = CurrBehavior.maxVelocity;
//	m_TargetAcceleration = desired_acceleration;
//	m_PrevSpeedError = e_v;
//	m_PrevDistanceError = e_d;
//	m_DesiredDistance = desired_distance;
//	m_DesiredSafeDistance = safe_follow_distance;
	m_PredictedVelMinusRealVel = m_ffEstimatedVelocity - CurrStatus.speed;
}

void MotionControl::VelocityControllerUpdateUsingInternalOpenPlannerACC(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVelocity, PlannerHNS::SHIFT_POS& desiredShift)
{
	desiredVelocity = CurrBehavior.maxVelocity;
	desiredShift = PlannerHNS::SHIFT_POS_DD;

	m_TargetSpeed = CurrBehavior.maxVelocity;
//	m_TargetAcceleration = desired_acceleration;
//	m_PrevSpeedError = e_v;
//	m_PrevDistanceError = e_d;
//	m_DesiredDistance = desired_distance;
//	m_DesiredSafeDistance = safe_follow_distance;
	m_PredictedVelMinusRealVel = m_ffEstimatedVelocity - CurrStatus.speed;
}

PlannerHNS::ExtendedVehicleState MotionControl::DoOneStep(const double& dt, const PlannerHNS::BehaviorState& behavior,
		const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose,
		const PlannerHNS::VehicleState& vehicleState, const bool& bNewTrajectory)
{
	if(bNewTrajectory && path.size() > 0)
	{
		UpdateCurrentPath(path);
		m_iPrevWayPoint = -1;
	}

	//calculate acceleration for logging and tuning
	double dv = vehicleState.speed - m_PrevSpeed;
	m_PrevSpeed = vehicleState.speed;
	if(dv != 0 && dt != 0)
	{
		m_InstantAcceleration = dv/dt;
	}
	m_AccelerationSum += m_InstantAcceleration;
	m_nAccelerations++;

	PlannerHNS::ExtendedVehicleState desiredState;
	if(m_HyperParams.bEnableCalibration)
	{
		CalibrationStep(dt, vehicleState, desiredState.steer, desiredState.speed);
		desiredState.shift = PlannerHNS::SHIFT_POS_DD;
	}
	else if(m_Path.size()>0 && behavior.state != INITIAL_STATE )
	{

		FindNextWayPoint(m_Path, currPose, vehicleState.speed, m_FollowMePoint, m_PerpendicularPoint, m_LateralError, m_FollowingDistance);

		if(m_HyperParams.bEnableSteeringFF)
		{
			double curr_steering_angle = vehicleState.steer;
			PlanningHelpers::EstimateFuturePosition(currPose, curr_steering_angle, m_FollowingDistance*m_HyperParams.feed_forward_ratio,
					m_HyperParams.path_density/2.0, m_VehicleInfo.wheel_base, m_ForwardSimulationPoint);
		}
		else
		{
			m_ForwardSimulationPoint = currPose;
		}

		if(m_HyperParams.bEnableVelocityFF)
		{
			m_ffEstimatedVelocity = EstimateFutureVelocity(vehicleState.speed, m_TargetSpeed, m_PrevDesiredState.accel_stroke, m_PrevDesiredState.brake_stroke, dt);
		}
		else
		{
			m_ffEstimatedVelocity = vehicleState.speed;
		}

		if(m_HyperParams.bEnableVelocityMode)
		{
			if(m_HyperParams.bUseInternalACC)
			{
				VelocityControllerUpdateUsingInternalOpenPlannerACC(dt, vehicleState, behavior, desiredState.speed, desiredState.shift);
			}
			else
			{
				if(m_HyperParams.bEnableConstantVelocity)
				{
					desiredState.accel_stroke = m_VehicleInfo.max_accel_value;
					desiredState.brake_stroke = 0;
					desiredState.speed = m_VehicleInfo.max_speed_forward;
					desiredState.shift = PlannerHNS::SHIFT_POS_DD;
					m_TargetSpeed = desiredState.speed;
				}
				else
				{
					VelocityControllerUpdateUsingACC(dt, vehicleState, behavior, desiredState.speed, desiredState.shift);
				}
			}
		}
		else
		{
			if(m_HyperParams.bUseInternalACC)
			{
				StrokeControllerUpdateForOpenPlannerInternalACC(dt, vehicleState, behavior, desiredState.accel_stroke, desiredState.brake_stroke, desiredState.shift);
			}
			else
			{
				if(m_HyperParams.bEnableConstantVelocity)
				{
					desiredState.accel_stroke = m_VehicleInfo.max_accel_value;
					desiredState.brake_stroke = 0;
					desiredState.speed = m_VehicleInfo.max_speed_forward;
					desiredState.shift = PlannerHNS::SHIFT_POS_DD;
					m_TargetSpeed = desiredState.speed;
				}
				else
				{
					StrokeControllerUpdateTwoPID(dt, vehicleState, behavior, desiredState.accel_stroke, desiredState.brake_stroke, desiredState.shift);
				}
			}
		}

		if(m_HyperParams.bEnableSteeringMode)
		{
			SteerControllerUpdate(dt, m_ForwardSimulationPoint, m_FollowMePoint, vehicleState, behavior, m_LateralError, desiredState.steer);
		}
		else
		{
			TorqueControllerUpdate(dt, m_ForwardSimulationPoint, m_FollowMePoint, vehicleState, behavior, m_LateralError, desiredState.steer_torque);
		}

		//std::cout << "Current Steer: " << vehicleState.steer << ", Target A: " << m_TargetAngle << ", Error: " << m_PrevAngleError << ", Target Steer: " << desiredState.steer << std::endl;

		desiredState.target_accel = m_TargetAcceleration;
	}
	else
	{
		desiredState.steer = 0;
		desiredState.speed = 0;
		desiredState.shift = PlannerHNS::SHIFT_POS_DD;
		desiredState.steer_torque = 0;
		desiredState.accel_stroke = 0;
		desiredState.brake_stroke = 0;
		desiredState.target_accel = 0;
	}

	if(m_HyperParams.bEnableLogs == true && m_Path.size() > 0)
	{
		LogControlData(dt, behavior, currPose, vehicleState, desiredState);
	}

	m_PrevBehaviorStatus = behavior;
	m_PrevDesiredState = desiredState;

	return desiredState;
}

PlannerHNS::VehicleState MotionControl::DoOneSimulationStep(const double& dt, const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose,
				const PlannerHNS::VehicleState& vehicleState, const bool& bNewTrajectory)
{
	if(bNewTrajectory && path.size() > 0)
	{
		UpdateCurrentPath(path);
		m_iPrevWayPoint = -1;
	}

	PlannerHNS::VehicleState desiredState;
	PlannerHNS::BehaviorState behavior;
	behavior.state = FORWARD_STATE;

	if(m_Path.size() > 1)
	{
		FindNextWayPoint(m_Path, currPose, vehicleState.speed, m_FollowMePoint, m_PerpendicularPoint, m_LateralError, m_FollowingDistance);

		if(m_HyperParams.bEnableSteeringFF)
		{
			double curr_steering_angle = vehicleState.steer;
			PlanningHelpers::EstimateFuturePosition(currPose, curr_steering_angle, m_FollowingDistance*m_HyperParams.feed_forward_ratio,
					m_HyperParams.path_density/2.0, m_VehicleInfo.wheel_base, m_ForwardSimulationPoint);
		}
		else
		{
			m_ForwardSimulationPoint = currPose;
		}

		if(m_HyperParams.bEnableConstantVelocity)
		{
			desiredState.speed = behavior.maxVelocity = m_VehicleInfo.max_speed_forward;
			desiredState.shift = PlannerHNS::SHIFT_POS_DD;
		}
		else
		{
			VelocityControllerUpdateUsingACC(dt, vehicleState, behavior, desiredState.speed, desiredState.shift);
		}

		SteerControllerUpdate(dt, m_ForwardSimulationPoint, m_FollowMePoint, vehicleState, behavior, m_LateralError, desiredState.steer);
	}
	else
	{
		desiredState.steer = 0;
		desiredState.speed = 0;
		desiredState.shift = PlannerHNS::SHIFT_POS_DD;
	}

	m_PrevBehaviorStatus = behavior;

	return desiredState;
}

void MotionControl::LogControlData(const double& dt, const PlannerHNS::BehaviorState& behavior, const PlannerHNS::WayPoint& currPose,
			const PlannerHNS::VehicleState& vehicleState, const PlannerHNS::ExtendedVehicleState& desiredState)
{
	double predictedRelativeSpeed = behavior.followVelocity - vehicleState.speed;

	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	double time_total = UtilityHNS::UtilityH::GetTimeDiffNow(m_LogTimer);
	std::ostringstream time_str;
	time_str.precision(4);
	time_str << time_total;
	std::ostringstream dataLine;
	dataLine << dt << "," << time_str.str() << "," << currPose.pos.x << "," << currPose.pos.y << "," <<
			currPose.pos.a << "," << m_TargetAngle << "," << m_PrevAngleError << "," << desiredState.steer_torque << "," <<
			behavior.state << "," << behavior.stopDistance << "," << m_InstantAcceleration << "," << m_AverageRelativeSpeed << "," << predictedRelativeSpeed << "," <<
			vehicleState.speed << "," << m_TargetSpeed << "," << m_TargetAcceleration << "," << m_PrevSpeedError << "," << m_PrevDistanceError << "," << m_DesiredDistance << "," << m_DesiredSafeDistance << "," <<
			desiredState.accel_stroke << "," <<	desiredState.brake_stroke <<  "," << m_LateralError << "," << m_iPrevWayPoint << "," << m_Path.size() << "," << m_AverageAcceleration <<"," << m_TotalAcceleration << "," <<
			m_ffEstimatedVelocity << "," << m_PredictedVelMinusRealVel << ",";
	m_LogData.push_back(dataLine.str());

	if(m_HyperParams.bEnableCalibration)
	{
		LogCalibrationData(vehicleState, desiredState);
	}

	if(m_HyperParams.bEnableSteeringMode)
	{
		m_LogSteerPIDData.push_back(m_pidSteering.ToString());
	}
	else
	{
		m_LogTorquePIDData.push_back(m_pidSteerWheelTorque.ToString());
	}

	if(m_HyperParams.bEnableVelocityMode)
	{
		//m_LogVelocityPIDData.push_back(m_pidVelocity.ToString());
	}
	else
	{
		m_LogAccelerationPIDData.push_back(m_pidAccelPedal.ToString());
		m_LogBrakingPIDData.push_back(m_pidBrakePedal.ToString());
	}

	m_LogFollowPIDData.push_back(m_pidFollow.ToString());

	/**
	 * These variable used in the logs
	 */

	bool bEnoughEvidence = CalcAvgRelativeSpeedFromDistance(dt, behavior, m_AverageRelativeSpeed);
}

bool MotionControl::CalcAvgRelativeSpeedFromDistance(const double& dt, const PlannerHNS::BehaviorState& CurrBehavior, double avg_relative_speed)
{
	bool bEnoughEvidence = false;
	if(CurrBehavior.state == FOLLOW_STATE)
	{
		if(m_RelativeSpeeds.size() > 0)
		{
			double relative_instant_velocity = (CurrBehavior.followDistance - m_PrevFollowDistance)/dt;
			m_RelativeSpeeds.push_back(relative_instant_velocity);

			if(m_RelativeSpeeds.size() > 10)
			{
				double vel_sum = 0;
				for(auto& v: m_RelativeSpeeds)
				{
					vel_sum += v;
				}
				avg_relative_speed = vel_sum / m_RelativeSpeeds.size();
				m_RelativeSpeeds.erase(m_RelativeSpeeds.begin()+0);
				bEnoughEvidence = true;
			}
		}
		else
		{
			m_RelativeSpeeds.push_back(0);
		}

		m_PrevFollowDistance = CurrBehavior.followDistance;
	}
	else
	{
		m_PrevFollowDistance = 0;
		m_RelativeSpeeds.clear();
	}

	return bEnoughEvidence;
}

void MotionControl::CalibrationStep(const double& dt, const PlannerHNS::VehicleState& CurrStatus, double& desiredSteer, double& desiredVelocity)
{
	if(m_iNextTest >= (int)m_CalibrationRunList.size()-1)
	{
		desiredSteer = 0;
		desiredVelocity = 0;
		return;
	}

	if(fabs(CurrStatus.speed - m_CalibrationRunList.at(m_iNextTest).first)*3.6 <= 1
			&& fabs(CurrStatus.steer - m_CalibrationRunList.at(m_iNextTest).second)*RAD2DEG <=0.5)
	{
		m_iNextTest++;
	}

	desiredVelocity = m_CalibrationRunList.at(m_iNextTest).first;
	desiredSteer = m_CalibrationRunList.at(m_iNextTest).second;

	std::cout << "i:" << m_iNextTest << ", desVel:" << desiredVelocity << ", CurVel:" << CurrStatus.speed
			<< ", desStr:" << desiredSteer << ", CurrStr:" << CurrStatus.steer << std::endl;

//	double e = targetSpeed - CurrStatus.speed;
//	if(e >= 0)
//		desiredVelocity = (m_VehicleInfo.max_acceleration * dt) + CurrStatus.speed;
//	else
//		desiredVelocity = (m_VehicleInfo.max_deceleration * dt) + CurrStatus.speed;
}

void MotionControl::LogCalibrationData(const PlannerHNS::VehicleState& currState,const PlannerHNS::VehicleState& desiredState)
{
	int startAngle=0, finishAngle=0, originalTargetAngle=0, currVelocity = 0;
	double t_FromStartToFinish_a = 0;
	bool bAngleReset = false;
	int startV=0, finishV=0, originalTargetV=0, currSteering = 0;
	double t_FromStartToFinish_v = 0;
	bool bVelocityReset = false;

	//1- decide reset
	if((int)(m_prevDesiredState_steer.steer*RAD2DEG) != (int)(desiredState.steer*RAD2DEG))
		bAngleReset = true;

	if((int)(m_prevDesiredState_vel.speed*3.6) != (int)(desiredState.speed*3.6))
		bVelocityReset = true;

	//2- calculate time and log
	if(bAngleReset)
	{
		startAngle = m_prevCurrState_steer.steer*RAD2DEG;
		finishAngle = currState.steer*RAD2DEG;
		originalTargetAngle = m_prevDesiredState_steer.steer*RAD2DEG;
		t_FromStartToFinish_a = UtilityHNS::UtilityH::GetTimeDiffNow(m_SteerDelayTimer);
		currVelocity = currState.speed*3.6;
		UtilityHNS::UtilityH::GetTickCount(m_SteerDelayTimer);

		std::ostringstream dataLine;
		dataLine << UtilityHNS::UtilityH::GetLongTime(m_SteerDelayTimer) << ","
				<< bAngleReset << ","
				<< startAngle << ","
				<< finishAngle << ","
				<< originalTargetAngle << ","
				<< t_FromStartToFinish_a << ","
				<< currVelocity << ",";

		m_SteerCalibrationData.push_back(dataLine.str());

		if(bAngleReset)
		{
			bAngleReset = false;
			m_prevCurrState_steer = currState;
			m_prevDesiredState_steer = desiredState;
		}
	}

	if(bVelocityReset)
	{
		startV = m_prevCurrState_vel.speed*3.6;
		finishV = currState.speed*3.6;
		originalTargetV = m_prevDesiredState_vel.speed*3.6;
		t_FromStartToFinish_v = UtilityHNS::UtilityH::GetTimeDiffNow(m_VelocityDelayTimer);
		currSteering = currState.steer*RAD2DEG;
		UtilityHNS::UtilityH::GetTickCount(m_VelocityDelayTimer);

		std::ostringstream dataLine;
		dataLine << UtilityHNS::UtilityH::GetLongTime(m_VelocityDelayTimer) << ","
				<< bVelocityReset << ","
				<< startV << ","
				<< finishV << ","
				<< originalTargetV << ","
				<< t_FromStartToFinish_v << ","
				<< currSteering << ",";

		m_VelocityCalibrationData.push_back(dataLine.str());

		if(bVelocityReset)
		{
			bVelocityReset = false;
			m_prevCurrState_vel = currState;
			m_prevDesiredState_vel = desiredState;
		}
	}
}

void MotionControl::CoordinateAscent(double tolerance, PID_CONST& pOut)
{

//   double p[3] = {0,0,0};
//   double pd[3] = {1,1,1};
//
//   int N = 100;
//   float change_factor = 0.01;
//   CAR_BASIC_INFO* inf =  m_Config->GetCarBasicInfo();
//   inf->velocity_pid.kP=p[0];
//   inf->velocity_pid.kI=p[1];
//   inf->velocity_pid.kD=p[2];
//   double err = 0;
//   double best_error=0, e=0;
//
//	pCurrBehavior->Start();
//	for(int j=0;j<N*2; j++)
//	{
//		StepMoveToObjective();
//		if(j>=N)
//			e += pCurrBehavior->m_SpeedErr*pCurrBehavior->m_SpeedErr;
//	}
//	best_error = e/(double)N;
//
//    int n = 0;
//	while( tolerance < (pd[0]+pd[1]+pd[2]))
//	{
//        for(int i=0; i< 3; i++)
//		{
//            p[i] += pd[i];
//            e=0;
//			inf->velocity_pid.kP=p[0];
//			inf->velocity_pid.kI=p[1];
//			inf->velocity_pid.kD=p[2];
//			ResetPlanner();
//			pCurrBehavior->Start();
//			for(int j=0;j<N*2; j++)
//			{
//				StepMoveToObjective(true);
//				if(j>=N)
//					e += pCurrBehavior->m_SpeedErr*pCurrBehavior->m_SpeedErr;
//			}
//
//			err = e/(double)N;
//			if( err < best_error)
//			{
//                best_error = err;
//               pd[i] *= change_factor;
//			}
//            else
//			{
//                p[i] -= (2 * pd[i]);
//
//				e=0;
//				inf->velocity_pid.kP=p[0];
//				inf->velocity_pid.kI=p[1];
//				inf->velocity_pid.kD=p[2];
//
//				ResetPlanner();
//				pCurrBehavior->Start();
//				for(int j=0;j<N*2; j++)
//				{
//					StepMoveToObjective(true);
//					if(j>=N)
//						e += pCurrBehavior->m_SpeedErr*pCurrBehavior->m_SpeedErr;
//				}
//                err = e/(double)N;
//                if (err < best_error)
//				{
//                    best_error = err;
//                    pd[i] *= change_factor;
//				}
//                else
//				{
//                    p[i] += pd[i];
//                    pd[i] *= (1-change_factor);
//				}
//			}
//		}
//		n+=1;
//
//
//	}
//	pOut.kP = p[0];
//	pOut.kI = p[1];
//	pOut.kD = p[2];

}

void MotionControl::InitCalibration()
{
	m_CalibrationRunList.push_back(std::make_pair(0,0));
	m_CalibrationRunList.push_back(std::make_pair(0,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(0,0.0));
	m_CalibrationRunList.push_back(std::make_pair(0,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(0,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(0,0.0));
	m_CalibrationRunList.push_back(std::make_pair(0,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(0,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(0,0.0));
	m_CalibrationRunList.push_back(std::make_pair(0,-m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(0,m_VehicleInfo.max_wheel_angle/1.0));
	m_CalibrationRunList.push_back(std::make_pair(0,0.0));
	m_CalibrationRunList.push_back(std::make_pair(0,-m_VehicleInfo.max_wheel_angle/1.0));

	m_CalibrationRunList.push_back(std::make_pair(1,0));
	m_CalibrationRunList.push_back(std::make_pair(1,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(1,0.0));
	m_CalibrationRunList.push_back(std::make_pair(1,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(1,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(1,0.0));
	m_CalibrationRunList.push_back(std::make_pair(1,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(1,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(1,0.0));
	m_CalibrationRunList.push_back(std::make_pair(1,-m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(1,m_VehicleInfo.max_wheel_angle/1.0));
	m_CalibrationRunList.push_back(std::make_pair(1,0.0));
	m_CalibrationRunList.push_back(std::make_pair(1,-m_VehicleInfo.max_wheel_angle/1.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(2,0));
	m_CalibrationRunList.push_back(std::make_pair(2,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(2,0.0));
	m_CalibrationRunList.push_back(std::make_pair(2,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(2,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(2,0.0));
	m_CalibrationRunList.push_back(std::make_pair(2,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(2,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(2,0.0));
	m_CalibrationRunList.push_back(std::make_pair(2,-m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(2,m_VehicleInfo.max_wheel_angle/1.0));
	m_CalibrationRunList.push_back(std::make_pair(2,0.0));
	m_CalibrationRunList.push_back(std::make_pair(2,-m_VehicleInfo.max_wheel_angle/1.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(3,0));
	m_CalibrationRunList.push_back(std::make_pair(3,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(3,0.0));
	m_CalibrationRunList.push_back(std::make_pair(3,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(3,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(3,0.0));
	m_CalibrationRunList.push_back(std::make_pair(3,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(3,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(3,0.0));
	m_CalibrationRunList.push_back(std::make_pair(3,-m_VehicleInfo.max_wheel_angle/1.5));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(4,0));
	m_CalibrationRunList.push_back(std::make_pair(4,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(4,0.0));
	m_CalibrationRunList.push_back(std::make_pair(4,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(4,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(4,0.0));
	m_CalibrationRunList.push_back(std::make_pair(4,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(4,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(4,0.0));
	m_CalibrationRunList.push_back(std::make_pair(4,-m_VehicleInfo.max_wheel_angle/1.5));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(5,0));
	m_CalibrationRunList.push_back(std::make_pair(5,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(5,0.0));
	m_CalibrationRunList.push_back(std::make_pair(5,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(5,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(5,0.0));
	m_CalibrationRunList.push_back(std::make_pair(5,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(5,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(5,0.0));
	m_CalibrationRunList.push_back(std::make_pair(5,-m_VehicleInfo.max_wheel_angle/1.5));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(6,0));
	m_CalibrationRunList.push_back(std::make_pair(6,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(6,0.0));
	m_CalibrationRunList.push_back(std::make_pair(6,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(6,m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(6,0.0));
	m_CalibrationRunList.push_back(std::make_pair(6,-m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(6,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(6,0.0));
	m_CalibrationRunList.push_back(std::make_pair(6,-m_VehicleInfo.max_wheel_angle/2.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(7,0));
	m_CalibrationRunList.push_back(std::make_pair(7,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(7,0.0));
	m_CalibrationRunList.push_back(std::make_pair(7,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(7,m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(7,0.0));
	m_CalibrationRunList.push_back(std::make_pair(7,-m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(7,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(7,0.0));
	m_CalibrationRunList.push_back(std::make_pair(7,-m_VehicleInfo.max_wheel_angle/2.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(8,0));
	m_CalibrationRunList.push_back(std::make_pair(8,m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(8,0.0));
	m_CalibrationRunList.push_back(std::make_pair(8,-m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(8,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(8,0.0));
	m_CalibrationRunList.push_back(std::make_pair(8,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(8,m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(8,0.0));
	m_CalibrationRunList.push_back(std::make_pair(8,-m_VehicleInfo.max_wheel_angle/3.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(9,0));
	m_CalibrationRunList.push_back(std::make_pair(9,m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(9,0.0));
	m_CalibrationRunList.push_back(std::make_pair(9,-m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(9,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(9,0.0));
	m_CalibrationRunList.push_back(std::make_pair(9,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(9,m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(9,0.0));
	m_CalibrationRunList.push_back(std::make_pair(9,-m_VehicleInfo.max_wheel_angle/3.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(10,0));
	m_CalibrationRunList.push_back(std::make_pair(10,m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(10,0.0));
	m_CalibrationRunList.push_back(std::make_pair(10,-m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(10,m_VehicleInfo.max_wheel_angle/5.0));
	m_CalibrationRunList.push_back(std::make_pair(10,0.0));
	m_CalibrationRunList.push_back(std::make_pair(10,-m_VehicleInfo.max_wheel_angle/5.0));
	m_CalibrationRunList.push_back(std::make_pair(10,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(10,0.0));
	m_CalibrationRunList.push_back(std::make_pair(10,-m_VehicleInfo.max_wheel_angle/4.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(11,0));
	m_CalibrationRunList.push_back(std::make_pair(11,m_VehicleInfo.max_wheel_angle/8.0));
	m_CalibrationRunList.push_back(std::make_pair(11,0.0));
	m_CalibrationRunList.push_back(std::make_pair(11,-m_VehicleInfo.max_wheel_angle/8.0));
	m_CalibrationRunList.push_back(std::make_pair(11,m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(11,0.0));
	m_CalibrationRunList.push_back(std::make_pair(11,-m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(11,m_VehicleInfo.max_wheel_angle/5.0));
	m_CalibrationRunList.push_back(std::make_pair(11,0.0));
	m_CalibrationRunList.push_back(std::make_pair(11,-m_VehicleInfo.max_wheel_angle/5.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(13,0));
	m_CalibrationRunList.push_back(std::make_pair(13,m_VehicleInfo.max_wheel_angle/10.0));
	m_CalibrationRunList.push_back(std::make_pair(13,0.0));
	m_CalibrationRunList.push_back(std::make_pair(13,-m_VehicleInfo.max_wheel_angle/10.0));
	m_CalibrationRunList.push_back(std::make_pair(13,m_VehicleInfo.max_wheel_angle/9.0));
	m_CalibrationRunList.push_back(std::make_pair(13,0.0));
	m_CalibrationRunList.push_back(std::make_pair(13,-m_VehicleInfo.max_wheel_angle/9.0));
	m_CalibrationRunList.push_back(std::make_pair(13,m_VehicleInfo.max_wheel_angle/8.0));
	m_CalibrationRunList.push_back(std::make_pair(13,0.0));
	m_CalibrationRunList.push_back(std::make_pair(13,-m_VehicleInfo.max_wheel_angle/8.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(15,0));
	m_CalibrationRunList.push_back(std::make_pair(15,m_VehicleInfo.max_wheel_angle/15.0));
	m_CalibrationRunList.push_back(std::make_pair(15,0.0));
	m_CalibrationRunList.push_back(std::make_pair(15,-m_VehicleInfo.max_wheel_angle/15.0));
	m_CalibrationRunList.push_back(std::make_pair(15,m_VehicleInfo.max_wheel_angle/12.0));
	m_CalibrationRunList.push_back(std::make_pair(15,0.0));
	m_CalibrationRunList.push_back(std::make_pair(15,-m_VehicleInfo.max_wheel_angle/12.0));
	m_CalibrationRunList.push_back(std::make_pair(15,m_VehicleInfo.max_wheel_angle/10.0));
	m_CalibrationRunList.push_back(std::make_pair(15,0.0));
	m_CalibrationRunList.push_back(std::make_pair(15,-m_VehicleInfo.max_wheel_angle/10.0));
	m_CalibrationRunList.push_back(std::make_pair(0,0));
}

} /* namespace SimulationNS */
