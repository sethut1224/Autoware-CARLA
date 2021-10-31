
/// \file op_controller.h
/// \brief PID based trajectory follower and velocity controller
/// \author Hatem Darweesh
/// \date July 04, 2020


#ifndef MOTION_CONTROLLER_H_
#define MOTION_CONTROLLER_H_
#include "op_planner/RoadNetwork.h"
#include "op_utility/UtilityH.h"
#include "op_planner/PlannerCommonDef.h"

namespace PlannerHNS
{

#define MAX_DECELERSTION -3*9.8 // m/s*s , max deceleration value is 3G

class ExtendedVehicleState : public VehicleState
{
public:
	double steer_torque = 0;
	double accel_stroke = 0;
	double brake_stroke = 0;
	double target_accel = 0;
};

class ControllerHyperParams
{
public:
	std::string log_file_path;
	double follow_velocity_gain_ratio = 1.0;
	double feed_forward_ratio = 0.75;
	double path_density = 0.5;
	bool bEnableConstantVelocity = true;
	bool bEnableVelocityMode = true;
	bool bEnableSteeringMode = true;
	bool bEnableVelocityFF = false; // should only used when there is acceleration delay, used with Stroke Mode controller
	bool bEnableSteeringFF = false; // should only used when there is steering delay, used with both Torque and Steering mode controllers
	bool bEnableLogs = false; // Write a log file for each controller part beside the whole controller log file
	bool bEnableCalibration = false; //Try to estimate the steering delay values limits
	bool bUseInternalACC = false; //only available when velocity mode is enabled. if enabled velocity values calculated internally in OpenPlanner will be used. if not the target velocity will be calculated in this control loop
};

class MotionControl
{
public:
	MotionControl();
	virtual ~MotionControl();

	void UpdateCurrentPath(const std::vector<PlannerHNS::WayPoint>& path);

	void TorqueControllerUpdate(const double& dt, const PlannerHNS::WayPoint& CurrPose, const PlannerHNS::WayPoint& TargetPose,
			const PlannerHNS::VehicleState& CurrStatus, const PlannerHNS::BehaviorState& CurrBehavior,
			const double& lateralErr, double& desiredSteerTorque);

	void SteerControllerUpdate(const double& dt, const PlannerHNS::WayPoint& CurrPose, const PlannerHNS::WayPoint& TargetPose,
				const PlannerHNS::VehicleState& CurrStatus, const PlannerHNS::BehaviorState& CurrBehavior,
				const double& lateralErr, double& desiredSteerAngle);

	int StrokeControllerUpdateTwoPID(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredAccel, double& desiredBrake, PlannerHNS::SHIFT_POS& desiredShift);

	int StrokeControllerUpdateForOpenPlannerInternalACC(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredAccel, double& desiredBrake, PlannerHNS::SHIFT_POS& desiredShift);

	void Init(const PlannerHNS::ControllerParams& params, const ControllerHyperParams& hyper_params, const PlannerHNS::CAR_BASIC_INFO& vehicleInfo);

	PlannerHNS::ExtendedVehicleState DoOneStep(const double& dt, const PlannerHNS::BehaviorState& behavior,
				const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose,
				const PlannerHNS::VehicleState& vehicleState, const bool& bNewTrajectory);

	PlannerHNS::VehicleState DoOneSimulationStep(const double& dt, const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose,
					const PlannerHNS::VehicleState& vehicleState, const bool& bNewTrajectory);

	void ResetLogTime(const double& v0, const double& v1);

	//Testing Points
	PlannerHNS::WayPoint m_PerpendicularPoint; // on track point, parallel to vehicle
	PlannerHNS::WayPoint m_FollowMePoint;
	PlannerHNS::WayPoint m_ForwardSimulationPoint;
	double m_FollowingDistance;
	//std::string m_ExperimentFolderName;
	//bool m_bUseInternalOpACC;

private:
	ControllerHyperParams m_HyperParams;
	PlannerHNS::ControllerParams m_Params;
	PlannerHNS::CAR_BASIC_INFO m_VehicleInfo;
	std::vector<PlannerHNS::WayPoint> m_Path;
	double m_PrevDesiredTorque; // control output
	double m_PrevDesiredSteerAngle; // control output
	PlannerHNS::BehaviorState m_PrevBehaviorStatus;
	double m_PrevFollowDistance;
	std::vector<double> m_RelativeSpeeds;
	double m_AverageRelativeSpeed;
	double m_PrevAngleError;
	PlannerHNS::ExtendedVehicleState m_PrevDesiredState;
	double m_ffEstimatedVelocity;
	double m_PredictedVelMinusRealVel;

	/**
	 * Log Information
	 */
	int m_iPrevWayPoint;
	double m_LateralError;
	double m_TargetAngle;
	double m_TargetSpeed;
	double m_PrevSpeedError;
	double m_PrevDistanceError;
	double m_PrevSpeed;
	double m_InstantAcceleration;
	double m_DesiredDistance;
	double m_DesiredSafeDistance;
	double m_AverageAcceleration;
	double m_TotalAcceleration;
	double m_AccelerationSum;
	int m_nAccelerations;
	double m_TargetAcceleration;

	UtilityHNS::PIDController m_pidSteering;

	UtilityHNS::PIDController m_pidSteerWheelTorque;
	UtilityHNS::PIDController m_pidAccelPedal;
	UtilityHNS::PIDController m_pidBrakePedal;
	UtilityHNS::PIDController m_pidFollow;

	//bool m_bEnableLog;
	std::vector<std::string> m_LogData;
	std::vector<std::string> m_LogSteerPIDData;
	std::vector<std::string> m_LogTorquePIDData;
	std::vector<std::string> m_LogVelocityPIDData;
	std::vector<std::string> m_LogFollowPIDData;
	std::vector<std::string> m_LogAccelerationPIDData;
	std::vector<std::string> m_LogBrakingPIDData;

	//Steering and Velocity Calibration Global Variables
	//bool m_bSteeringMode;
	//bool m_bVelocityMode;
	//bool m_bCalibrationMode;
	int	m_iNextTest;
	std::vector<std::string> m_SteerCalibrationData;
	std::vector<std::string> m_VelocityCalibrationData;
	PlannerHNS::VehicleState m_prevCurrState_steer;
	PlannerHNS::VehicleState m_prevDesiredState_steer;
	PlannerHNS::VehicleState m_prevCurrState_vel;
	PlannerHNS::VehicleState m_prevDesiredState_vel;
	struct timespec m_SteerDelayTimer;
	struct timespec m_VelocityDelayTimer;
	struct timespec m_LogTimer;
	std::vector<std::pair<double, double> > m_CalibrationRunList;

	bool FindNextWayPoint(const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
			const double& velocity, PlannerHNS::WayPoint& pursuite_point, PlannerHNS::WayPoint& prep,
			double& lateral_err, double& follow_distance);

	void TorqueControllerPart(const double& dt, const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
			const double& lateral_error, double& torque_d);

	void VelocityControllerUpdateUsingACC(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVelocity, PlannerHNS::SHIFT_POS& desiredShift);

	void VelocityControllerUpdateUsingInternalOpenPlannerACC(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
				const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVelocity, PlannerHNS::SHIFT_POS& desiredShift);

	void AngleControllerPart(const double& dt, const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
			const double& lateral_error, double& steer_angle_d);

	void PredictMotion(double& x, double &y, double& heading, double steering, double velocity,
			double wheelbase, double time_elapsed);

	double EstimateFutureVelocity(double v0, double v_d, double accel_stroke, double brake_stroke, double time_elapsed);

//	void EstimateFuturePosition(const PlannerHNS::WayPoint& currPose, const double& currSteering, const double& distance,
//			PlannerHNS::WayPoint& estimatedPose);

	double GetPID_LinearChange(double minVal, double maxVal, double speedMax, double currSpeed);

	void CalculateVelocityDesired(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& vel_d, double& acc_d, double& distance_d, double& safe_d);

	void LogCalibrationData(const PlannerHNS::VehicleState& currState,const PlannerHNS::VehicleState& desiredState);
	void InitCalibration();
	void CalibrationStep(const double& dt, const PlannerHNS::VehicleState& CurrStatus, double& desiredSteer, double& desiredVelocity);
	void CoordinateAscent(double tolerance, PID_CONST& pOut);
	bool CalcAvgRelativeSpeedFromDistance(const double& dt, const PlannerHNS::BehaviorState& CurrBehavior, double avg_relative_speed);
	double CalculateLookAheadDistance(const double& steering_delay, const double& curr_velocity, const double& min_distance);

	void LogControlData(const double& dt, const PlannerHNS::BehaviorState& behavior, const PlannerHNS::WayPoint& currPose,
			const PlannerHNS::VehicleState& vehicleState, const PlannerHNS::ExtendedVehicleState& desiredState);
};

} /* namespace PlannerHNS */

#endif /* MOTION_CONTROLLER_H_ */
