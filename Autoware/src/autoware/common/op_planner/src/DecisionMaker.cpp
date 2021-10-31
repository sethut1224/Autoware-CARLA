
/// \file DecisionMaker.cpp
/// \brief Initialize behaviors state machine, and calculate required parameters for the state machine transition conditions
/// \author Hatem Darweesh
/// \date Dec 14, 2016


#include "op_planner/DecisionMaker.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"


namespace PlannerHNS
{

DecisionMaker::DecisionMaker()
{
	m_iSinceLastReplan = 0;
	m_CurrGlobalId = -1;
	m_iCurrentTotalPathId = 0;
	//pLane = nullptr;
	m_pCurrentBehaviorState = nullptr;
	m_pGoToGoalState = nullptr;
	m_pWaitState= nullptr;
	m_pMissionCompleteState= nullptr;
	m_pAvoidObstacleState = nullptr;
	m_pTrafficLightStopState = nullptr;
	m_pTrafficLightWaitState = nullptr;
	m_pStopSignStopState = nullptr;
	m_pStopSignWaitState = nullptr;
	m_pFollowState = nullptr;
	m_pMissionCompleteState = nullptr;
	m_pGoalState = nullptr;
	m_pGoToGoalState = nullptr;
	m_pWaitState = nullptr;
	m_pInitState = nullptr;
	m_pFollowState = nullptr;
	m_pAvoidObstacleState = nullptr;
	m_pStopState = nullptr;
	m_bRequestNewGlobalPlan = false;
	m_bUseInternalACC = false;
}

DecisionMaker::~DecisionMaker()
{
	delete m_pStopState;
	delete m_pMissionCompleteState;
	delete m_pGoalState;
	delete m_pGoToGoalState;
	delete m_pWaitState;
	delete m_pInitState;
	delete m_pFollowState;
	delete m_pAvoidObstacleState;
	delete m_pTrafficLightStopState;
	delete m_pTrafficLightWaitState;
	delete m_pStopSignWaitState;
	delete m_pStopSignStopState;
}

void DecisionMaker::UpdateParameters(const ControllerParams& ctrlParams, const PlanningParams& params, const CAR_BASIC_INFO& carInfo)
{
	m_CarInfo = carInfo;
	m_ControlParams = ctrlParams;
	m_params = params;
}

void DecisionMaker::Init(const ControllerParams& ctrlParams, const PlannerHNS::PlanningParams& params,const CAR_BASIC_INFO& carInfo)
 	{
 		m_CarInfo = carInfo;
 		m_ControlParams = ctrlParams;
 		m_params = params;
 		//m_original_params = params;

 		//m_VelocityController.Init(m_ControlParams, m_CarInfo, true);

 		m_pidVelocity.Init(0.01, 0.004, 0.01);
		m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

		m_pidStopping.Init(0.005, 0.005, 0.01);
		m_pidStopping.Setlimit(m_params.horizonDistance, 0);

		m_pidFollowing.Init(0.05, 0.05, 0.01);
		m_pidFollowing.Setlimit(m_params.minFollowingDistance, 0);

		InitBehaviorStates();

		if(m_pCurrentBehaviorState)
		{
			m_pCurrentBehaviorState->SetBehaviorsParams(&m_params);
			PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();
			pValues->minStoppingDistance = m_params.horizonDistance;
			pValues->iCentralTrajectory = m_params.rollOutNumber/2;
			pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;
			pValues->iCurrSafeLane = 0;
			pValues->stoppingDistances.clear();
			pValues->stoppingDistances.push_back(m_params.horizonDistance);
			pValues->currentVelocity = 0;
			pValues->bTrafficIsRed = false;
			pValues->currentTrafficLightID = -1;
			pValues->currentStopSignID = -1;
			pValues->bFullyBlock = false;
			pValues->bFinalLocalTrajectory = false;
			pValues->distanceToNext = m_params.horizonDistance;
			pValues->velocityOfNext = 0;
			pValues->distanceToGoal = m_params.horizonDistance;
			pValues->currentGoalID = -1;
			pValues->prevGoalID = -1;
		}
 	}

void DecisionMaker::UpdateAvoidanceParams(bool enable_swerve, int roll_out_numbers)
{
	if(enable_swerve == false && enable_swerve != m_params.enableSwerving)
	{
		m_pCurrentBehaviorState->GetCalcParams()->bRePlan = true;
	}

	m_params.enableSwerving = enable_swerve;
	m_params.rollOutNumber = roll_out_numbers;

}

void DecisionMaker::InitBehaviorStates()
{

	m_pStopState = new StopStateII(&m_params, 0, 0);
	m_pMissionCompleteState = new MissionAccomplishedStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), 0);
	m_pGoalState = new GoalStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pMissionCompleteState);
	m_pGoToGoalState = new ForwardStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoalState);
	m_pInitState = new InitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

	m_pFollowState = new FollowStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pAvoidObstacleState = new SwerveStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pStopSignWaitState = new StopSignWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pStopSignStopState = new StopSignStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pStopSignWaitState);

	m_pTrafficLightWaitState = new TrafficLightWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pTrafficLightStopState = new TrafficLightStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

	m_pStopState->InsertNextState(m_pGoToGoalState);
	m_pStopState->InsertNextState(m_pGoalState);
	m_pStopState->decisionMakingCount = 0;

	m_pGoToGoalState->InsertNextState(m_pAvoidObstacleState);
	m_pGoToGoalState->InsertNextState(m_pStopSignStopState);
	m_pGoToGoalState->InsertNextState(m_pTrafficLightStopState);
	m_pGoToGoalState->InsertNextState(m_pFollowState);
	m_pGoToGoalState->InsertNextState(m_pStopState);
	m_pGoToGoalState->decisionMakingCount = 0;//m_params.nReliableCount;

	m_pGoalState->InsertNextState(m_pGoToGoalState);

	m_pStopSignWaitState->decisionMakingTime = m_params.stopSignStopTime;
	m_pStopSignWaitState->InsertNextState(m_pStopSignStopState);
	m_pStopSignWaitState->InsertNextState(m_pGoalState);


	m_pTrafficLightStopState->InsertNextState(m_pTrafficLightWaitState);

	m_pTrafficLightWaitState->InsertNextState(m_pTrafficLightStopState);
	m_pTrafficLightWaitState->InsertNextState(m_pGoalState);

	m_pFollowState->InsertNextState(m_pAvoidObstacleState);
	m_pFollowState->InsertNextState(m_pStopSignStopState);
	m_pFollowState->InsertNextState(m_pTrafficLightStopState);
	m_pFollowState->InsertNextState(m_pGoalState);
	m_pFollowState->decisionMakingCount = 0;//m_params.nReliableCount;

	m_pInitState->decisionMakingCount = 0;//m_params.nReliableCount;

	m_pCurrentBehaviorState = m_pInitState;
}

 bool DecisionMaker::GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<PlannerHNS::TrafficLight>& trafficLights, PlannerHNS::TrafficLight& trafficL)
 {
	 for(unsigned int i = 0; i < trafficLights.size(); i++)
	 {
		 double d = hypot(trafficLights.at(i).pose.pos.y - state.pos.y, trafficLights.at(i).pose.pos.x - state.pos.x);
		 if(d <= trafficLights.at(i).stoppingDistance)
		 {
			 double a_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(UtilityHNS::UtilityH::FixNegativeAngle(trafficLights.at(i).pose.pos.a) , UtilityHNS::UtilityH::FixNegativeAngle(state.pos.a));

			 if(a_diff < M_PI_2 && trafficLights.at(i).id != prevTrafficLightId)
			 {
				 //std::cout << "Detected Light, ID = " << trafficLights.at(i).id << ", Distance = " << d << ", Angle = " << trafficLights.at(i).pos.a*RAD2DEG << ", Car Heading = " << state.pos.a*RAD2DEG << ", Diff = " << a_diff*RAD2DEG << std::endl;
				 trafficL = trafficLights.at(i);
				 return true;
			 }
		 }
	 }

	 return false;
 }

 void DecisionMaker::CalculateImportantParameterForDecisionMaking(const PlannerHNS::VehicleState& car_state,
		 const bool& bEmergencyStop, const std::vector<TrafficLight>& detectedLights,
		 const TrajectoryCost& bestTrajectory)
 {
	 if(m_TotalPaths.size() == 0) return;

	//Initialize internal decision making state variables
 	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

 	if(m_CarInfo.max_deceleration != 0)
 	{
 		pValues->minStoppingDistance = -pow(car_state.speed, 2)/(2.0*m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance;
 	}
 	else
 	{
 		pValues->minStoppingDistance = m_params.horizonDistance;
 	}

 	pValues->stoppingDistances.clear();
 	pValues->stoppingDistances.push_back(m_params.horizonDistance);
 	pValues->stoppingDistances.push_back(pValues->minStoppingDistance);
 	pValues->currentVelocity = car_state.speed;
 	pValues->bTrafficIsRed = false;
 	pValues->currentTrafficLightID = -1;
 	pValues->bFullyBlock = false;
 	pValues->bFinalLocalTrajectory = false;
 	pValues->distanceToNext = bestTrajectory.closest_obj_distance;
 	pValues->velocityOfNext = bestTrajectory.closest_obj_velocity;


	/**
	 * Global Lanes section, set global path index and ID
	 */
 	//if(bestTrajectory.lane_index >= 0 && m_bRequestNewGlobalPlan == false)
 	if(bestTrajectory.lane_index >= 0 && bestTrajectory.lane_index < m_TotalPaths.size())
 	{
 		pValues->iCurrSafeLane = bestTrajectory.lane_index;
 	}
 	else
 	{
 		PlannerHNS::RelativeInfo info;
 		PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_TotalPaths, state, m_params.rollOutDensity*m_params.rollOutNumber/2.0 + 0.1, info);
 		pValues->iCurrSafeLane = info.iGlobalPath;
 	}

 	m_iCurrentTotalPathId = pValues->iCurrSafeLane;

	for(unsigned int ig=0; ig < m_TotalPaths.size(); ig++)
	{
		if(ig == m_iCurrentTotalPathId && m_TotalPaths.at(ig).size() > 0)
		{
			m_CurrGlobalId = m_TotalPaths.at(ig).at(0).gid;
		}
	}

	/**
	 * Block End ---------------------------------------------------------------------
	 */


	/**
	 * Local Trajectory section, set local trajectory index
	 */

	if(m_LanesRollOuts.at(m_iCurrentTotalPathId).size() <= 1)
	{
		m_params.rollOutNumber = 0;
//		m_params.enableSwerving = false;
	}
	else
	{
		m_params.rollOutNumber = m_LanesRollOuts.at(m_iCurrentTotalPathId).size() - 1;
	//	m_params.enableSwerving = m_original_params.enableSwerving;
	}

 	pValues->iCentralTrajectory	= m_pCurrentBehaviorState->m_pParams->rollOutNumber/2;

	if(pValues->iPrevSafeTrajectory < 0)
	{
		pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;
	}

 	if(bestTrajectory.index >=0 &&  bestTrajectory.index < (int)m_LanesRollOuts.at(m_iCurrentTotalPathId).size())
 	{
 		pValues->iCurrSafeTrajectory = bestTrajectory.index;
 	}
 	else
 	{
 		pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;
 	}

 	pValues->bFullyBlock = bestTrajectory.bBlocked;

	/**
	 * Block End ---------------------------------------------------------------------
	 */

	/**
	 * Set reach goal state values
	 */
 	pValues->distanceToGoal = PlannerHNS::PlanningHelpers::GetDistanceFromPoseToEnd(state, m_TotalPaths.at(pValues->iCurrSafeLane));
 	if((pValues->distanceToGoal < -m_params.goalDiscoveryDistance) || (pValues->distanceToGoal > m_params.horizonDistance))
 	{
 		pValues->distanceToGoal = m_params.horizonDistance;
 	}

 	pValues->stoppingDistances.push_back(pValues->distanceToGoal);

 	if(pValues->distanceToGoal < m_params.goalDiscoveryDistance)
 	{
 		pValues->currentGoalID = -1;
 	}
	else
	{
		pValues->currentGoalID = 1;
		pValues->prevGoalID = 1;
	}
	/**
	 * Block End ---------------------------------------------------------------------
	 */


	/**
	 * Set Traffic light and stop sign values
	 */
 	int stopLineID = -1;
 	int stopSignID = -1;
 	int trafficLightID = -1;
 	double distanceToClosestStopLine = 0;
 	bool bGreenTrafficLight = true;
 	double critical_long_front_distance =  m_params.additionalBrakingDistance + m_params.verticalSafetyDistance;

  	distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(m_TotalPaths.at(pValues->iCurrSafeLane), state, m_params.giveUpDistance, stopLineID, stopSignID, trafficLightID) - critical_long_front_distance;

 	if(distanceToClosestStopLine > m_params.giveUpDistance && distanceToClosestStopLine < (pValues->minStoppingDistance + 1.0))
 	{
 		if(m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior)
 		{
 			pValues->currentTrafficLightID = trafficLightID;
 			//Debug
 			//std::cout << "Detected Traffic Light: " << trafficLightID << std::endl;
 			for(unsigned int i=0; i< detectedLights.size(); i++)
 			{
 				if(detectedLights.at(i).id == trafficLightID)
 					bGreenTrafficLight = (detectedLights.at(i).lightType == GREEN_LIGHT);
 			}
 		}

 		if(m_pCurrentBehaviorState->m_pParams->enableStopSignBehavior || m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior)
 		{
 			pValues->currentStopSignID = stopSignID;
 			pValues->stoppingDistances.push_back(distanceToClosestStopLine);
 			//Debug
 			//std::cout << "LP => D: " << pValues->distanceToStop() << ", PrevSignID: " << pValues->prevTrafficLightID << ", CurrSignID: " << pValues->currentTrafficLightID << ", Green: " << bGreenTrafficLight << std::endl;
 		}
 	}

 	//Debug
 	//std::cout << "Distance To Closest: " << distanceToClosestStopLine << ", Stop LineID: " << stopLineID << ", Stop SignID: " << stopSignID << ", TFID: " << trafficLightID << std::endl;

 	pValues->bTrafficIsRed = !bGreenTrafficLight;

	/**
	 * Block End ---------------------------------------------------------------------
	 */

	/**
	 * Decide, Emergency stop values
	 */
 	if(bEmergencyStop)
	{
		pValues->bFullyBlock = true;
		pValues->distanceToNext = 1;
		pValues->velocityOfNext = 0;
	}
	/**
	 * Block End ---------------------------------------------------------------------
	 */


	/**
	 * Decide, Not to re-plan when reaching the final part of the global path
	 */
 	if(m_Path.size() > 0 && m_TotalOriginalPaths.size() > 0)
	{
		double d_between_ends = hypot(m_TotalOriginalPaths.at(m_iCurrentTotalPathId).back().pos.y - m_Path.back().pos.y, m_TotalOriginalPaths.at(m_iCurrentTotalPathId).back().pos.x - m_Path.back().pos.x);
		if(d_between_ends < m_params.pathDensity)
		{
			pValues->bFinalLocalTrajectory = true;
		}
	}
	/**
	 * Block End ---------------------------------------------------------------------
	 */

	/**
	 * Decide, request new global plan, when lane change is not possible
	 */
 	if(m_TotalPaths.size() > 1)
 	{
		for(unsigned int i=0; i < m_TotalPaths.size(); i++)
		{
			RelativeInfo curr_total_path_inf;
			PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(i), state, curr_total_path_inf);
			pValues->distanceToChangeLane = m_TotalPaths.at(i).back().distanceCost - curr_total_path_inf.perp_point.distanceCost;
			//if((pValues->distanceToChangeLane < m_params.microPlanDistance*0.75) || (fabs(curr_total_path_inf.perp_distance) < 1.0 && m_iCurrentTotalPathId == i)) commit 5f7e394 issue
			if(pValues->distanceToChangeLane < m_params.microPlanDistance*0.75)
			{
				m_bRequestNewGlobalPlan = true;
			}
		}
 	}
	/**
	 * Block End ---------------------------------------------------------------------
	 */


	/**
	 * Decide, select new local trajectory just before the tight curve, set CURVATURE_COST_UPPER_LIMIT and CURVATURE_COST_LOWER_LIMIT to zero to disable this feature
	 */
 	double curr_curve_cost = 0, min_curve_cost = 0;
 	CheckForCurveZone(car_state, curr_curve_cost, min_curve_cost);
 	if(curr_curve_cost >= CURVATURE_COST_UPPER_LIMIT && min_curve_cost < CURVATURE_COST_LOWER_LIMIT && pValues->bInsideCurveZone == false)
	{
 		pValues->bInsideCurveZone = true;
 		pValues->bRePlan = true;
 		//Debug
// 		std::cout << " ---------------------------------------------- " << std::endl;
//		std::cout << "Selecting premature plan .. " << curr_curve_cost << ", min_curv_ahead: " <<  min_curve_cost << std::endl;
//		std::cout << " ---------------------------------------------- " << std::endl;
	}
	else if(curr_curve_cost < CURVATURE_COST_LOWER_LIMIT && pValues->bInsideCurveZone == true)
	{
		pValues->bInsideCurveZone = false;
		//Debug
//		std::cout << " ---------------------------------------------- " << std::endl;
//		std::cout << "Exit curve.. " << curr_curve_cost << ", min_curv_ahead: " <<  min_curve_cost << std::endl;
//		std::cout << " ---------------------------------------------- " << std::endl;
	}
	/**
	 * Block End ---------------------------------------------------------------------
	 */

  if (!m_bUseInternalACC) {
    ComputeEgoFollowingAndStoppingVelocities(pValues, critical_long_front_distance);
  }
 }

 void DecisionMaker::ComputeEgoFollowingAndStoppingVelocities(PreCalculatedConditions* pValues, double &critical_long_front_distance)
 {
   /**
  * Computing Ego following velocity.
  */
     if (!pValues->bFullyBlock)
     {
         pValues->egoFollowingVelocity = m_params.maxSpeed;
     }
     else
     {
         // clip small speeds of object
         double objectVelocity = 0.0;
         if (pValues->velocityOfNext > 2)
             objectVelocity = pValues->velocityOfNext;

         // object speed dependent - in higher speeds keeps bigger distance
         double normalDistance =
                 pValues->distanceToNext - m_params.additionalBrakingDistance;

         double under_sqrt = objectVelocity * objectVelocity - 2 * m_CarInfo.max_deceleration * normalDistance;
         if (under_sqrt > 0)
             pValues->egoFollowingVelocity = sqrt(under_sqrt);
         else
             pValues->egoFollowingVelocity = 0.0;
     }
     // clip ego FOLLOW velocity between 0 and maxSpeed
     pValues->egoFollowingVelocity = std::min(std::max(pValues->egoFollowingVelocity, 0.0), m_params.maxSpeed);

     /**
      * Computing Ego stopping velocity.
      */
     if (pValues->distanceToStop() < 0) {
         pValues->egoStoppingVelocity = 0;
     }
     else {
         pValues->egoStoppingVelocity = sqrt(2 * abs(m_CarInfo.max_deceleration) * std::max(pValues->distanceToStop() - critical_long_front_distance, 0.));
     }
     // clip ego STOPPING velocity between 0 and maxSpeed
     pValues->egoStoppingVelocity = std::min(std::max(pValues->egoStoppingVelocity, 0.0), m_params.maxSpeed);
 }

 bool DecisionMaker::ReachEndOfGlobalPath(const double& min_distance, const int& iGlobalPathIndex)
 {
	 if(m_TotalPaths.size()==0) return false;

	 PlannerHNS::RelativeInfo info;
	 PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(iGlobalPathIndex), state, info);

	 double d = 0;
	 for(unsigned int i = info.iFront; i < m_TotalPaths.at(iGlobalPathIndex).size()-1; i++)
	 {
		 d+= hypot(m_TotalPaths.at(iGlobalPathIndex).at(i+1).pos.y - m_TotalPaths.at(iGlobalPathIndex).at(i).pos.y, m_TotalPaths.at(iGlobalPathIndex).at(i+1).pos.x - m_TotalPaths.at(iGlobalPathIndex).at(i).pos.x);
		 if(d > min_distance)
			 return false;
	 }

	 return true;
 }

 void DecisionMaker::SetNewGlobalPath(const std::vector<std::vector<WayPoint> >& globalPath)
 {
	 if(m_pCurrentBehaviorState)
	 {
		 m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = true;
		 m_bRequestNewGlobalPlan = false;
		 m_TotalOriginalPaths = globalPath;
		 m_prev_index.clear();
		 for(unsigned int i=0; i < globalPath.size(); i++)
		 {
			 m_prev_index.push_back(0);
		 }
	 }
 }

 void DecisionMaker::CheckForCurveZone(const VehicleState& vehicleState, double& curr_curve_cost, double& min_curve_cost)
 {
	double look_ahead_distance = PlannerHNS::PlanningHelpers::CalculateLookAheadDistance(m_ControlParams.SteeringDelay, vehicleState.speed, m_ControlParams.minPursuiteDistance) + m_CarInfo.length;
	RelativeInfo total_info;
	PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(m_iCurrentTotalPathId), state, total_info);
	curr_curve_cost = total_info.perp_point.curvatureCost;
	min_curve_cost = PlannerHNS::PlanningHelpers::GetCurvatureCostAhead(m_TotalPaths.at(m_iCurrentTotalPathId), total_info, total_info.iBack, look_ahead_distance);
 }

 bool DecisionMaker::TestForReplanningParams(const VehicleState& vehicleState)
 {

	PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();
	if(!preCalcPrams || m_LanesRollOuts.at(m_iCurrentTotalPathId).size() == 0) return false;
	int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_Path, state);
	int index_limit = m_Path.size()/2.0 + 1;

	if((currIndex > index_limit
			|| preCalcPrams->bRePlan
			|| preCalcPrams->bNewGlobalPath) && !preCalcPrams->bFinalLocalTrajectory && m_iSinceLastReplan > m_params.nReliableCount)
	{
		m_iSinceLastReplan = 0;
		return true;
	}

	return false;
 }

 bool DecisionMaker::SelectSafeTrajectory(const PlannerHNS::VehicleState& vehicleState)
 {
	 m_iSinceLastReplan++;
	 bool bNewTrajectory = false;
	 PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	 if(!preCalcPrams || m_LanesRollOuts.at(m_iCurrentTotalPathId).size() == 0) return bNewTrajectory;

	int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_Path, state);
	int index_limit = m_Path.size()/2.0 + 1;

	if((currIndex > index_limit
			|| preCalcPrams->bRePlan
			|| preCalcPrams->bNewGlobalPath) && !preCalcPrams->bFinalLocalTrajectory && m_iSinceLastReplan > m_params.nReliableCount)
	{
		//Debug
		//std::cout << "New Local Plan !! " << currIndex << ", "<< preCalcPrams->bRePlan << ", " << preCalcPrams->bNewGlobalPath  << ", " <<  m_TotalPath.at(0).size() << ", PrevLocal: " << m_Path.size();
		m_Path = m_LanesRollOuts.at(m_iCurrentTotalPathId).at(preCalcPrams->iCurrSafeTrajectory);
		//Debug
		//std::cout << ", NewLocal: " << m_Path.size() << std::endl;
		preCalcPrams->bNewGlobalPath = false;
		preCalcPrams->bRePlan = false;
		m_iSinceLastReplan = 0;
		bNewTrajectory = true;
	}

	return bNewTrajectory;
 }

 PlannerHNS::BehaviorState DecisionMaker::GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState)
 {
	PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	m_pCurrentBehaviorState = m_pCurrentBehaviorState->GetNextState();
	if(m_pCurrentBehaviorState==0)
		m_pCurrentBehaviorState = m_pInitState;

	PlannerHNS::BehaviorState currentBehavior;

	currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
	currentBehavior.followDistance = preCalcPrams->distanceToNext;

	currentBehavior.minVelocity		= 0;
	currentBehavior.stopDistance 	= preCalcPrams->distanceToStop();
	currentBehavior.followVelocity 	= preCalcPrams->velocityOfNext;
	if(preCalcPrams->iPrevSafeTrajectory<0 || preCalcPrams->iPrevSafeTrajectory >= m_LanesRollOuts.at(m_iCurrentTotalPathId).size())
	{
		currentBehavior.iTrajectory		= preCalcPrams->iCurrSafeTrajectory;
	}
	else
	{
		currentBehavior.iTrajectory		= preCalcPrams->iPrevSafeTrajectory;
	}

	currentBehavior.iLane = m_iCurrentTotalPathId;

	//double average_braking_distance = -pow(vehicleState.speed, 2)/(m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance; // average_braking_distance replaced by minStoppingDistance on 28th August 2018
	double indication_distance = preCalcPrams->minStoppingDistance;
	if(indication_distance  < m_params.minIndicationDistance)
	{
		indication_distance = m_params.minIndicationDistance;
	}

	currentBehavior.indicator = PlanningHelpers::GetIndicatorsFromPath(m_Path, state, indication_distance);
	if(currentBehavior.state == GOAL_STATE || currentBehavior.state == FINISH_STATE || m_params.maxSpeed == 0)
	{
	  currentBehavior.indicator = INDICATOR_BOTH;
	}

	return currentBehavior;
 }

 double DecisionMaker::UpdateVelocityDirectlyToTrajectorySmooth(BehaviorState& beh, const VehicleState& CurrStatus, const double& dt)
 {

	 PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	 if(!preCalcPrams || m_TotalPaths.size() == 0) return 0;

	 BehaviorState beh_with_max = beh;

	RelativeInfo total_info;
	PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(m_iCurrentTotalPathId), state, total_info);
	beh_with_max.maxVelocity = PlannerHNS::PlanningHelpers::GetVelocityAhead(m_TotalPaths.at(m_iCurrentTotalPathId), total_info, total_info.iBack, preCalcPrams->minStoppingDistance*m_params.curveSlowDownRatio);
	if(beh_with_max.maxVelocity > m_params.maxSpeed)
	{
		beh_with_max.maxVelocity = m_params.maxSpeed;
	}

	//There are another function for ACC in op_acc.cpp and op_controller. maybe they are better, maybe they are not ! only time can tell.
	double target_velocity = PlanningHelpers::GetACCVelocityModelBased(dt, CurrStatus.speed, m_CarInfo, m_ControlParams, beh_with_max);

	for(unsigned int i =  0; i < m_Path.size(); i++)
	{
		m_Path.at(i).v = target_velocity;
	}

	return target_velocity;
 }
 
 double DecisionMaker::UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt)
 {
 PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	if(!preCalcPrams || m_TotalPaths.size() == 0) return 0;

	RelativeInfo info, total_info;
	PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(m_iCurrentTotalPathId), state, total_info);
	PlanningHelpers::GetRelativeInfo(m_Path, state, info);
	double max_velocity	= PlannerHNS::PlanningHelpers::GetVelocityAhead(m_TotalPaths.at(m_iCurrentTotalPathId), total_info, total_info.iBack, preCalcPrams->minStoppingDistance*m_params.curveSlowDownRatio);
	if(max_velocity > m_params.maxSpeed)
	{
		max_velocity = m_params.maxSpeed;
	}

	//std::cout << "Max Velocity : " << max_velocity << "," << m_params.maxSpeed << std::endl;

	double critical_long_front_distance = m_CarInfo.length/2.0;
	double desiredVelocity = 0;

	if(beh.state == STOPPING_STATE || beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE)
	{
		double deceleration_critical = 0;
		double distance_to_stop = beh.stopDistance ;
		if(distance_to_stop != 0)
			deceleration_critical = (-CurrStatus.speed*CurrStatus.speed)/(2.0*distance_to_stop);

		if(deceleration_critical >= 0)
			deceleration_critical = m_CarInfo.max_deceleration;

		desiredVelocity = deceleration_critical * dt + CurrStatus.speed;

		//std::cout << "Stopping : V: " << CurrStatus.speed << ", A: " << deceleration_critical << ", dt: " << dt << std::endl;
		//std::cout << "Stopping (beh, brake): (" << beh.stopDistance << ", " << preCalcPrams->minStoppingDistance << ") , desiredPID=" << desiredVelocity << ", To Goal: " << preCalcPrams->distanceToGoal <<  std::endl;
	}
	else if(beh.state == FOLLOW_STATE)
	{

		double deceleration_critical = 0;
		double distance_to_stop = beh.followDistance -  critical_long_front_distance - m_params.additionalBrakingDistance;
		double sudden_stop_distance = -pow((CurrStatus.speed - beh.followVelocity), 2)/m_CarInfo.max_deceleration;

		if(distance_to_stop != 0)
			deceleration_critical = (-CurrStatus.speed*CurrStatus.speed)/(2.0*distance_to_stop);

		if(deceleration_critical >= 0)
			deceleration_critical = m_CarInfo.max_deceleration;

		desiredVelocity = deceleration_critical * dt + CurrStatus.speed;

//		if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory != m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
//		{
//			desiredVelocity  = desiredVelocity * 0.75;
//		}


		if(beh.followVelocity > CurrStatus.speed)
			desiredVelocity = CurrStatus.speed;


		//std::cout << "Following V: " << CurrStatus.speed << ", Desired V: " << beh.followVelocity << ", A: " << deceleration_critical << ", d_to_stop: " << distance_to_stop << ", sudden_stop_d" << sudden_stop_distance << std::endl;
		//std::cout << "Desired Vel: " << desiredVelocity << std::endl;

	}
	else if(beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE )
	{

		double acceleration_critical = m_CarInfo.max_acceleration;

		if(max_velocity < CurrStatus.speed)
			acceleration_critical = m_CarInfo.max_deceleration ;

		desiredVelocity = (acceleration_critical * dt) + CurrStatus.speed;

		desiredVelocity  = max_velocity;

		//std::cout << "bEnd : " << preCalcPrams->bFinalLocalTrajectory << ", Min D: " << preCalcPrams->minStoppingDistance << ", D To Goal: " << preCalcPrams->distanceToGoal << std::endl;
		//std::cout << "Forward: dt" << dt << " ,Target vel: " << desiredVelocity << ", Acc: " << acceleration_critical << ", Max Vel: " << max_velocity << ", Curr Vel: " << CurrStatus.speed << ", break_d: " << m_params.additionalBrakingDistance  << std::endl;
		//std::cout << "Forward Target Acc: " << acceleration_critical  << ", PID Velocity: " << desiredVelocity << ", Max Velocity : " << max_velocity  << std::endl;
	}
	else if(beh.state == STOP_SIGN_WAIT_STATE || beh.state == TRAFFIC_LIGHT_WAIT_STATE)
	{
		desiredVelocity = 0;
	}
	else
	{
		desiredVelocity = 0;
	}


	if(desiredVelocity >  m_params.maxSpeed)
	{
		desiredVelocity = m_params.maxSpeed;
	}
	else if(desiredVelocity < 0)
	{
		desiredVelocity = 0;
	}

	for(unsigned int i =  0; i < m_Path.size(); i++)
	{
		m_Path.at(i).v = desiredVelocity;
	}

	//This line where Tartu University changes take effect, currently causes the vehicle to go backwards
	//return std::min({max_velocity, preCalcPrams->egoStoppingVelocity, preCalcPrams->egoFollowingVelocity});
	return max_velocity;

 }

 PlannerHNS::BehaviorState DecisionMaker::DoOneStep(
		const double& dt,
		const PlannerHNS::WayPoint currPose,
		const PlannerHNS::VehicleState& vehicleState,
		const std::vector<TrafficLight>& trafficLight,
		const TrajectoryCost& tc,
		const bool& bEmergencyStop)
{
	 PlannerHNS::BehaviorState beh;
	 state = currPose;
	 m_TotalPaths.clear();

	for(unsigned int i = 0; i < m_TotalOriginalPaths.size(); i++)
	{
		t_centerTrajectorySmoothed.clear();
		m_prev_index.at(i) = PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_TotalOriginalPaths.at(i), state, m_params.horizonDistance ,	m_params.pathDensity , t_centerTrajectorySmoothed, m_prev_index.at(i));
		if(m_prev_index.at(i) > 0 ) m_prev_index.at(i) = m_prev_index.at(i) -1;
		m_TotalPaths.push_back(t_centerTrajectorySmoothed);
	}

	if(m_TotalPaths.size() == 0) return beh;

	CalculateImportantParameterForDecisionMaking(vehicleState, bEmergencyStop, trafficLight, tc);

	beh = GenerateBehaviorState(vehicleState);

	beh.bNewPlan = SelectSafeTrajectory(vehicleState);

	if(m_bUseInternalACC)
	{
		beh.maxVelocity = UpdateVelocityDirectlyToTrajectorySmooth(beh, vehicleState, dt);
	}
	else
	{
		beh.maxVelocity = UpdateVelocityDirectlyToTrajectory(beh, vehicleState, dt);
	}

	//Debug
	//std::cout << "Evaluated Rollouts size: " << m_LanesRollOuts.size() << std::endl;
	//std::cout << "Eval_i: " << tc.index << ", Curr_i: " <<  m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << ", Prev_i: " << m_pCurrentBehaviorState->GetCalcParams()->iPrevSafeTrajectory << std::endl;

	return beh;
 }

} /* namespace PlannerHNS */
