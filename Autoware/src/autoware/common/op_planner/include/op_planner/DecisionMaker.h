
/// \file DecisionMaker.h
/// \brief Initialize behaviors state machine, and calculate required parameters for the state machine transition conditions
/// \author Hatem Darweesh
/// \date Dec 14, 2016


#ifndef BEHAVIOR_DECISION_MAKER
#define BEHAVIOR_DECISION_MAKER

#include "op_planner/BehaviorStateMachine.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/RoadNetwork.h"
//#include "op_planner/control/op_acc.h"

namespace PlannerHNS
{

#define CURVATURE_COST_UPPER_LIMIT 0.98 //min = 0 , max = 1.0
#define CURVATURE_COST_LOWER_LIMIT 0.90 //min = 0 , max = 1.0

class DecisionMaker
{
public:
	WayPoint state;
	CAR_BASIC_INFO m_CarInfo;
	ControllerParams m_ControlParams;
	std::vector<WayPoint> m_Path;
	PlannerHNS::RoadNetwork m_Map;

	int m_iCurrentTotalPathId;
	int m_CurrGlobalId;
	std::vector<std::vector<std::vector<WayPoint> > > m_LanesRollOuts;
	std::vector<int> m_prev_index;
	int m_iSinceLastReplan;
	//Lane* pLane;

	BehaviorStateMachine* m_pCurrentBehaviorState;
	WaitState* m_pWaitState;
	SwerveStateII* m_pAvoidObstacleState;
	TrafficLightStopStateII* m_pTrafficLightStopState;
	TrafficLightWaitStateII* m_pTrafficLightWaitState;

	ForwardStateII* m_pGoToGoalState;;
	InitStateII* m_pInitState;
	MissionAccomplishedStateII* m_pMissionCompleteState;
	GoalStateII* m_pGoalState;
	FollowStateII* m_pFollowState;
	StopSignStopStateII* m_pStopSignStopState;
	StopSignWaitStateII* m_pStopSignWaitState;
	StopStateII* m_pStopState;

	void InitBehaviorStates();

	//For Simulation
	UtilityHNS::PIDController m_pidVelocity;
	UtilityHNS::PIDController m_pidStopping;
	UtilityHNS::PIDController m_pidFollowing;

	bool m_bRequestNewGlobalPlan;
	bool m_bUseInternalACC;

public:

	DecisionMaker();
	virtual ~DecisionMaker();
	void Init(const ControllerParams& ctrlParams, const PlanningParams& params, const CAR_BASIC_INFO& carInfo);
	void UpdateAvoidanceParams(bool enable_swerve, int roll_out_numbers);
	void UpdateParameters(const ControllerParams& ctrlParams, const PlanningParams& params, const CAR_BASIC_INFO& carInfo);
	void CalculateImportantParameterForDecisionMaking(const VehicleState& car_state,
			const bool& bEmergencyStop, const std::vector<TrafficLight>& detectedLights,
			const TrajectoryCost& bestTrajectory);
	void SetNewGlobalPath(const std::vector<std::vector<WayPoint> >& globalPath);

	BehaviorState DoOneStep(
			const double& dt,
			const PlannerHNS::WayPoint currPose,
			const PlannerHNS::VehicleState& vehicleState,
			const std::vector<TrafficLight>& trafficLight,
			const TrajectoryCost& tc,
			const bool& bEmergencyStop);

protected:
	bool GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<TrafficLight>& trafficLights, TrafficLight& trafficL);
	//void UpdateCurrentLane(const double& search_distance);
	bool SelectSafeTrajectory(const PlannerHNS::VehicleState& vehicleState);
	BehaviorState GenerateBehaviorState(const VehicleState& vehicleState);

	/**
	 * Sets the trajectory velocity to the raw velocity. Returns the max velocity calculated with the ego stopping and following velocity.
	 * Only without internal ECC.
	 * @param beh
	 * @param CurrStatus
	 * @param dt
	 */
	double UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt);

	/**
	 * Sets the trajectory velocity to a smoothed velocity. Returns the max velocity.
	 * Used with internal ECC.
	 * @param beh
	 * @param CurrStatus
	 * @param dt
	 */
	double UpdateVelocityDirectlyToTrajectorySmooth(BehaviorState& beh, const VehicleState& CurrStatus, const double& dt);

	/**
	 * Computes EgoFollowing and EgoStopping velocities. These values are used for decision making when the internal ECC is disabled.
	 * Details : https://github.com/hatem-darweesh/common/pull/9
	 * @param pValues Precalculated values
	 * @param critical_long_front_distance
	 */
  void ComputeEgoFollowingAndStoppingVelocities(PreCalculatedConditions* pValues, double &critical_long_front_distance);

	bool ReachEndOfGlobalPath(const double& min_distance, const int& iGlobalPathIndex);
	bool TestForReplanningParams(const VehicleState& vehicleState);
	void CheckForCurveZone(const VehicleState& vehicleState, double& curr_curve_cost, double& min_curve_cost);


	std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
	std::vector<std::vector<WayPoint> > m_TotalOriginalPaths;
	std::vector<std::vector<WayPoint> > m_TotalPaths;
	PlannerHNS::PlanningParams m_params;
	//PlannerHNS::PlanningParams m_original_params;
	//ACC m_VelocityController;


};

} /* namespace PlannerHNS */

#endif /* BEHAVIOR_DECISION_MAKER */
