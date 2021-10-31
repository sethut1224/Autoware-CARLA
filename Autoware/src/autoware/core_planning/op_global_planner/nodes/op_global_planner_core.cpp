/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "op_global_planner_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/KmlMapLoader.h"
#include "op_planner/OpenDriveMapLoader.h"
#include "op_planner/Lanelet2MapLoader.h"
#include "op_planner/VectorMapLoader.h"

namespace GlobalPlanningNS
{

GlobalPlanner::GlobalPlanner()
{
	m_ClearCostTime = 200; // 2 hours before clearing the collision points
	m_iMessageID = 1;
	m_pCurrGoal = 0;
	m_iCurrentGoalIndex = 0;
	m_HMIDestinationID = 0;
	m_bFirstStartHMI = false;
	m_bWaitingState = false;
	m_bSlowDownState = false;
	m_bStoppingState = false;
	m_bReStartState = false;
	m_bDestinationError = false;
	m_bReplanSignal = false;
	m_GlobalPathID = 1;
	m_bStart = false;
	UtilityHNS::UtilityH::GetTickCount(m_WaitingTimer);
	UtilityHNS::UtilityH::GetTickCount(m_ReplanningTimer);


	nh.getParam("/op_global_planner/pathDensity" , m_params.pathDensity);
	nh.getParam("/op_global_planner/enableSmoothing" , m_params.bEnableSmoothing);
	nh.getParam("/op_global_planner/enableLaneChange" , m_params.bEnableLaneChange);
	nh.getParam("/op_global_planner/enableRvizInput" , m_params.bEnableRvizInput);
	nh.getParam("/op_global_planner/enableHMI" , m_params.bEnableHMI);
	nh.getParam("/op_global_planner/experimentName" , m_params.exprimentName);
	if(m_params.exprimentName.size() > 0)
	{
		if(m_params.exprimentName.at(m_params.exprimentName.size()-1) != '/')
			m_params.exprimentName.push_back('/');
	}

	UtilityHNS::DataRW::CreateLoggingMainFolder();
	if(m_params.exprimentName.size() > 1)
	{
		UtilityHNS::DataRW::CreateExperimentFolder(m_params.exprimentName);
	}

	nh.getParam("/op_global_planner/goalConfirmDistance" , m_params.endOfPathDistance);
	nh.getParam("/op_global_planner/enableReplan" , m_params.bEnableReplanning);

	pub_Paths = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
	pub_PathsRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
	pub_MapRviz  = nh.advertise<visualization_msgs::MarkerArray>("vector_map_center_lines_rviz", 1, true);
	pub_GoalsListRviz = nh.advertise<visualization_msgs::MarkerArray>("op_destinations_rviz", 1, true);

	if(m_params.bEnableHMI)
	{
		sub_hmi_mission = nh.subscribe("/hmi_mission_command", 1, &GlobalPlanner::callbackGetHMIState, this);
		pub_hmi_mission = nh.advertise<autoware_msgs::State>("/op_mission_status", 1, true);
		nh.getParam("/op_global_planner/destinationFileName", m_params.destinationsFile);
		LoadDestinations(m_params.destinationsFile);
	}


	if(m_params.bEnableRvizInput)
	{
		sub_start_pose = nh.subscribe("/initialpose", 1, &GlobalPlanner::callbackGetStartPose, this);
		sub_goal_pose = nh.subscribe("move_base_simple/goal", 1, &GlobalPlanner::callbackGetGoalPose, this);
	}
	else
	{
		LoadSimulationData();
	}

	sub_v2x_obstacles = nh.subscribe("/op_v2x_replanning_signal", 1, &GlobalPlanner::callbackGetV2XReplanSignal, this);
	sub_replan_signal = nh.subscribe("/op_global_replan", 1, &GlobalPlanner::callbackGetReplanSignal, this);
	sub_current_pose = nh.subscribe("/current_pose", 1, &GlobalPlanner::callbackGetCurrentPose, this);

	m_VelHandler.InitVelocityHandler(nh, PlannerHNS::CAR_BASIC_INFO(), &m_VehicleState, &m_CurrentPose);


	m_MapHandler.InitMapHandler(nh, "/op_global_planner/mapSource",
			"/op_global_planner/mapFileName", "/op_global_planner/lanelet2_origin");

	tf::StampedTransform transform;
	tf::TransformListener tf_listener;
	PlannerHNS::ROSHelpers::getTransformFromTF("world", "map", tf_listener, transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();


	/**
	 * Animate Global path generation
	 */
	m_bEnableAnimation = false;
	m_CurrMaxCost = 1;
	m_iCurrLevel = 0;
	m_nLevelSize = 1;
	m_bSwitch = 0;
	pub_GlobalPlanAnimationRviz = nh.advertise<visualization_msgs::MarkerArray>("/op_global_path_animation", 1, true);
}

GlobalPlanner::~GlobalPlanner()
{
	if(m_params.bEnableRvizInput)
	{
		SaveSimulationData();
	}
}

void GlobalPlanner::callbackGetHMIState(const autoware_msgs::StateConstPtr& msg)
{
	//std::cout << "Received HMI Message .. " << msg->mission_state << std::endl;

	PlannerHNS::HMI_MSG inc_msg = PlannerHNS::HMI_MSG::FromString(msg->mission_state);
	if(inc_msg.type == PlannerHNS::CONFIRM_MSG) //client received destinations
	{
		std::cout << "Received Confirmation.. " << std::endl;
		m_bFirstStartHMI = true;
	}
	else if(inc_msg.type == PlannerHNS::DESTINATIONS_MSG) // Client is requesting destinations
	{
		std::cout << "Received Destinations Request .. " << std::endl;
		m_bFirstStartHMI = false;
		//send destinations and wait for confirmation
		LoadDestinations(m_params.destinationsFile);

		std::cout << "Send Destinations Data.. " << std::endl;
	}
	else if(inc_msg.type == PlannerHNS::COMMAND_MSG) // Client is sending a command
	{
		std::cout << "Received Command Request .. " << std::endl;

		if(inc_msg.current_action == PlannerHNS::MSG_START_ACTION || inc_msg.current_action == PlannerHNS::MSG_CHANGE_DESTINATION)
		{
			m_bFirstStartHMI = true;
			if(m_bSlowDownState || m_bStoppingState || m_iCurrentGoalIndex == 0)
			{
				m_bReStartState = true;
			}
			m_bSlowDownState = false;
			m_bStoppingState = false;
			m_HMIDestinationID = inc_msg.curr_destination_id;
			//std::cout << " >>>>> Go Go Go ! " <<  std::endl;
		}
		else if(inc_msg.current_action == PlannerHNS::MSG_SLOWDOWN_ACTION)
		{
			m_bSlowDownState = true;
		}
		else if(inc_msg.current_action == PlannerHNS::MSG_STOP_ACTION)
		{
			m_bStoppingState = true;
			m_bSlowDownState = false;
		}
	}
}

bool GlobalPlanner::UpdateGoalWithHMI()
{
	if(m_bStoppingState || m_bSlowDownState || m_bReStartState)
	{
		return true;
	}
	else if(m_GoalsPos.size() == 1)
	{
		if(m_GoalsPos.at(0).id == m_HMIDestinationID)
		{
			m_iCurrentGoalIndex = 0;
			return true;
		}
		else
		{
			return false;
		}
	}
	else if(m_GoalsPos.size() > 1)
	{
		if(m_iCurrentGoalIndex == ((int)m_GoalsPos.size()-1) && m_GoalsPos.at(0).id == m_HMIDestinationID) // Can only Go to first destination from last
		{
			if(m_bWaitingState)
			{
				m_iCurrentGoalIndex = 0;
				return true;
			}
			else
			{
				return false;
			}
		}

		for(unsigned int i=0; i < m_GoalsPos.size(); i++)
		{
			if(m_GoalsPos.at(i).id == m_HMIDestinationID && m_iCurrentGoalIndex < i) // Don't Go Backwards, don't change to same destination again
			{
				m_iCurrentGoalIndex = i;
				return true;
			}
		}
	}

	return false;
}

void GlobalPlanner::callbackGetReplanSignal(const std_msgs::BoolConstPtr& msg)
{
	m_bReplanSignal = msg->data;
}

void GlobalPlanner::callbackGetV2XReplanSignal(const geometry_msgs::PoseArrayConstPtr& msg)
{
	std::vector<PlannerHNS::WayPoint> points;
	for(auto& p: msg->poses)
	{
		points.push_back(PlannerHNS::WayPoint(p.position.x, p.position.y, p.position.z, 0));
	}

	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	std::vector<PlannerHNS::WayPoint*> modified_nodes;
	PlannerHNS::MappingHelpers::UpdateMapWithSignalPose(points, m_Map, modified_nodes, 0.75, -1);
	m_ModifiedMapItemsTimes.push_back(std::make_pair(modified_nodes, t));

	m_bReplanSignal = true;
}

void GlobalPlanner::ClearOldCostFromMap()
{
	for(int i=0; i < (int)m_ModifiedMapItemsTimes.size(); i++)
	{
		if(UtilityHNS::UtilityH::GetTimeDiffNow(m_ModifiedMapItemsTimes.at(i).second) > m_ClearCostTime)
		{
			for(unsigned int j= 0 ; j < m_ModifiedMapItemsTimes.at(i).first.size(); j++)
			{
				for(unsigned int i_action=0; i_action < m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.size(); i_action++)
				{
					if(m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.at(i_action).first == PlannerHNS::CHANGE_DESTINATION)
					{
						m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.at(i_action).second = 0;
					}
				}
			}

			m_ModifiedMapItemsTimes.erase(m_ModifiedMapItemsTimes.begin()+i);
			i--;
		}
	}
}

void GlobalPlanner::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	// if(m_GoalsPos.size() == 0)
	// {
		PlannerHNS::WayPoint wp = PlannerHNS::WayPoint(msg->pose.position.x+m_OriginPos.position.x, msg->pose.position.y+m_OriginPos.position.y, msg->pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.orientation));
		m_GoalsPos.push_back(wp);
		ROS_INFO("Received Goal Pose");
	// }
	
}

void GlobalPlanner::callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	m_CurrentPose = PlannerHNS::WayPoint(msg->pose.pose.position.x+m_OriginPos.position.x, msg->pose.pose.position.y+m_OriginPos.position.y, msg->pose.pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.pose.orientation));
	m_StartPose = m_CurrentPose;
	m_bStart = true;
	ROS_INFO("Received Start pose");
}

void GlobalPlanner::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPose.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	m_bStart = true;
}

bool GlobalPlanner::GenerateGlobalPlan(PlannerHNS::WayPoint& startPoint, PlannerHNS::WayPoint& goalPoint, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths)
{
	std::vector<int> predefinedLanesIds;
	double ret = 0;
	//The distance that is needed to brake from current speed to zero with acceleration of -1 m/s*s
	double planning_distance = pow((m_CurrentPose.v), 2);
	if(planning_distance < MIN_EXTRA_PLAN_DISTANCE)
	{
		planning_distance = MIN_EXTRA_PLAN_DISTANCE;
	}

	if(m_bEnableAnimation)
	{
		if(m_PlanningVisualizeTree.size() > 0)
		{
			m_PlannerH.DeleteWaypoints(m_PlanningVisualizeTree);
			m_AccumPlanLevels.markers.clear();
			m_iCurrLevel = 0;
			m_nLevelSize = 1;
		}

		std::vector<int> predefinedLanesIds;
		double ret = m_PlannerH.PlanUsingDP(startPoint, goalPoint,
				MAX_GLOBAL_PLAN_SEARCH_DISTANCE, planning_distance, m_params.bEnableLaneChange, predefinedLanesIds,
				m_Map, generatedTotalPaths, &m_PlanningVisualizeTree);

		m_pCurrGoal = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(goalPoint, m_Map);

		if(m_PlanningVisualizeTree.size() > 1)
		{
			m_CurrentLevel.push_back(m_PlanningVisualizeTree.at(0));
			m_CurrMaxCost = 0;
			for(auto& wp: m_PlanningVisualizeTree)
			{
				if(wp->cost > m_CurrMaxCost)
				{
					m_CurrMaxCost = wp->cost;
				}
			}
		}
	}
	else
	{
		if(m_bStoppingState)
		{
			generatedTotalPaths.clear();
			ret = m_PlannerH.PlanUsingDPRandom(startPoint, 20, m_Map, generatedTotalPaths);
		}
		else
		{
			ret = m_PlannerH.PlanUsingDP(startPoint, goalPoint, MAX_GLOBAL_PLAN_SEARCH_DISTANCE, planning_distance,  m_params.bEnableLaneChange, predefinedLanesIds, m_Map, generatedTotalPaths);
		}
	}

	if(ret == 0)
	{
		std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString()
												<< ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
		return false;
	}


	if(generatedTotalPaths.size() > 0 && generatedTotalPaths.at(0).size()>0)
	{
		if(m_params.bEnableSmoothing)
		{
			for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
			{
				PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_params.pathDensity);
				PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.1 , 0.1);
			}
		}
		else
		{
			for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
			{
				PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.05 , 0.1);
			}
		}

		m_prev_index.clear();
		for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
		{
			m_prev_index.push_back(0); // start following the global path from waypoint index 0

			PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
			if(m_GlobalPathID > 10000)
			{
				m_GlobalPathID = 1;
			}

			for(unsigned int j=0; j < generatedTotalPaths.at(i).size(); j++)
			{
				generatedTotalPaths.at(i).at(j).gid = m_GlobalPathID;
				if(m_bSlowDownState)
				{
					generatedTotalPaths.at(i).at(j).v = m_params.slowDownSpeed;
				}
			}

			m_GlobalPathID++;
			std::cout << "New DP Path -> " << generatedTotalPaths.at(i).size() << std::endl;
		}
		return true;
	}
	else
	{
		std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString() << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
	}
	return false;
}

void GlobalPlanner::FindIncommingBranches(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths, const PlannerHNS::WayPoint& currPose,const double& min_distance,const double& max_distance,
			std::vector<PlannerHNS::WayPoint*>& branches)
{
//	static int detection_range = 75; // meter
	if(globalPaths.size() > 0)
	{
		int close_index = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(globalPaths.at(0), currPose);
		PlannerHNS::WayPoint closest_wp = globalPaths.at(0).at(close_index);
		double d = 0;
		for(unsigned int i=close_index+1; i < globalPaths.at(0).size(); i++)
		{
			d += hypot(globalPaths.at(0).at(i).pos.y - globalPaths.at(0).at(i-1).pos.y, globalPaths.at(0).at(i).pos.x - globalPaths.at(0).at(i-1).pos.x);

//			if(d - min_distance > detection_range)
//				break;

			if(d < max_distance && d > min_distance && globalPaths.at(0).at(i).pFronts.size() > 1)
			{
				for(unsigned int j = 0; j< globalPaths.at(0).at(i).pFronts.size(); j++)
				{
					PlannerHNS::WayPoint* wp =  globalPaths.at(0).at(i).pFronts.at(j);


					bool bFound = false;
					for(unsigned int ib=0; ib< branches.size(); ib++)
					{
						if(wp->actionCost.size() > 0 &&  branches.at(ib)->actionCost.size() > 0 && branches.at(ib)->actionCost.at(0).first == wp->actionCost.at(0).first)
						{
							bFound = true;
							break;
						}
					}

					if(wp->actionCost.size() > 0 && !bFound && closest_wp.laneId != wp->laneId)
						branches.push_back(wp);
				}
			}
		}
	}
}

void GlobalPlanner::SendAvailableOptionsHMI()
{
	if(!m_bFirstStartHMI) return;

	std::vector<PlannerHNS::WayPoint*> branches;
	if(m_GeneratedTotalPaths.size()>0 && m_GeneratedTotalPaths.at(0).size()>0)
	{
		FindIncommingBranches(m_GeneratedTotalPaths,m_CurrentPose, 30, 75, branches);
	}

	//Options message contains either destination reached option or branches types options
	PlannerHNS::HMI_MSG msg;
	if(m_iMessageID > 9999999)
	{
		m_iMessageID = 1;
	}

	msg.msg_id = m_iMessageID++;
	msg.bErr = false;
	if(m_bDestinationError == true)
	{
		msg.bErr = true;
		std::ostringstream err_str;
		if(m_iCurrentGoalIndex < m_GoalsPos.size())
		{
			err_str << "Invalid request. Can't Change destination from: " << m_GoalsPos.at(m_iCurrentGoalIndex).id << " To: " << m_HMIDestinationID;
		}
		else
		{
			err_str << "Invalid operation. Destination index: " << m_iCurrentGoalIndex << " Out of range: " << m_GoalsPos.size();
		}
		msg.err_msg = err_str.str();
	}
	msg.type = PlannerHNS::OPTIONS_MSG;
	if(m_iCurrentGoalIndex >= 0 && m_iCurrentGoalIndex < m_GoalsPos.size())
	{
		msg.curr_destination_id = m_GoalsPos.at(m_iCurrentGoalIndex).id;
	}
	if(m_iCurrentGoalIndex+1 >= 0 && m_iCurrentGoalIndex+1 < m_GoalsPos.size())
	{
		msg.next_destination_id = m_GoalsPos.at(m_iCurrentGoalIndex+1).id;
	}

	if(branches.size() > 0)
	{
		for(unsigned int i = 0; i< branches.size(); i++)
		{
			msg.available_actions.push_back(ToMsgAction(branches.at(i)->actionCost.at(0).first));
		}

		//TODO use current selected global path index , not zero
		int close_index = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_GeneratedTotalPaths.at(0), m_CurrentPose);
		PlannerHNS::WayPoint* currOptions = 0;
		for(unsigned int i=close_index+1; i < m_GeneratedTotalPaths.at(0).size(); i++)
		{
			bool bFound = false;
			for(unsigned int j=0; j< branches.size(); j++)
			{
				if(branches.at(j)->id == m_GeneratedTotalPaths.at(0).at(i).id)
				{
					currOptions = branches.at(j);
					bFound = true;
					break;
				}
			}
			if(bFound)
				break;
		}

		if(currOptions !=0 )
		{
			msg.current_action = ToMsgAction(currOptions->actionCost.at(0).first);
			//msg.currID = currOptions->laneId;
		}
	}

	if(m_bWaitingState) //when waiting we can start from HMI
	{
		msg.available_actions.push_back(PlannerHNS::MSG_START_ACTION);
	}
	else //if there is an active plan, we can slow down, stop and change destination
	{
		msg.available_actions.push_back(PlannerHNS::MSG_SLOWDOWN_ACTION);
		msg.available_actions.push_back(PlannerHNS::MSG_STOP_ACTION);
		msg.available_actions.push_back(PlannerHNS::MSG_CHANGE_DESTINATION);
	}

	if(PlannerHNS::PlanningHelpers::CheckForEndOfPaths(m_GeneratedTotalPaths, m_CurrentPose, m_params.endOfPathDistance) >= 0 && m_VehicleState.speed < 0.25)
	{
		msg.available_actions.push_back(PlannerHNS::MSG_DESTINATION_REACHED);
	}

	autoware_msgs::State auto_state_msg;
	auto_state_msg.mission_state = msg.CreateStringMessage();
	pub_hmi_mission.publish(auto_state_msg);
}

void GlobalPlanner::AnimatedVisualizationForGlobalPath(double time_interval)
{
	if(UtilityHNS::UtilityH::GetTimeDiffNow(m_animation_timer) > time_interval)
	{
		UtilityHNS::UtilityH::GetTickCount(m_animation_timer);
		m_CurrentLevel.clear();

		for(unsigned int ilev = 0; ilev < m_nLevelSize && m_iCurrLevel < m_PlanningVisualizeTree.size() ; ilev ++)
		{
			m_CurrentLevel.push_back(m_PlanningVisualizeTree.at(m_iCurrLevel));
			m_nLevelSize += m_PlanningVisualizeTree.at(m_iCurrLevel)->pFronts.size() - 1;
			m_iCurrLevel++;
		}


		if(m_CurrentLevel.size() == 0 && m_GeneratedTotalPaths.size() > 0)
		{
			m_bSwitch++;
			m_AccumPlanLevels.markers.clear();

			if(m_bSwitch == 2)
			{
				for(unsigned int il = 0; il < m_GeneratedTotalPaths.size(); il++)
				{
					for(unsigned int ip = 0; ip < m_GeneratedTotalPaths.at(il).size(); ip ++)
					{
						m_CurrentLevel.push_back(&m_GeneratedTotalPaths.at(il).at(ip));
					}

				}
				std::cout << "Switch On " << std::endl;
				m_bSwitch = 0;
			}
			else
			{
				for(unsigned int ilev = 0; ilev < m_PlanningVisualizeTree.size()+200; ilev ++)
				{
					m_CurrentLevel.push_back(m_PlanningVisualizeTree.at(0));
				}
				std::cout << "Switch Off " << std::endl;
			}

			PlannerHNS::ROSHelpers::CreateNextPlanningTreeLevelMarker(m_CurrentLevel, m_AccumPlanLevels, m_pCurrGoal, m_CurrMaxCost);
			pub_GlobalPlanAnimationRviz.publish(m_AccumPlanLevels);
		}
		else
		{
			PlannerHNS::ROSHelpers::CreateNextPlanningTreeLevelMarker(m_CurrentLevel, m_AccumPlanLevels, m_pCurrGoal, m_CurrMaxCost);

			if(m_AccumPlanLevels.markers.size() > 0)
			{
				pub_GlobalPlanAnimationRviz.publish(m_AccumPlanLevels);
			}
		}
	}
}

void GlobalPlanner::VisualizeAndSend(const std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths)
{
	autoware_msgs::LaneArray lane_array;
	visualization_msgs::MarkerArray pathsToVisualize;

	for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
	{
		autoware_msgs::Lane lane;
		PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(generatedTotalPaths.at(i), lane);
		lane_array.lanes.push_back(lane);
	}

	std_msgs::ColorRGBA total_color;
	total_color.r = 0;
	total_color.g = 0.7;
	total_color.b = 1.0;
	total_color.a = 0.9;
	PlannerHNS::ROSHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
	PlannerHNS::ROSHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
	PlannerHNS::ROSHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);

	if((m_bFirstStartHMI && m_params.bEnableHMI) || !m_params.bEnableHMI)
	{
		pub_PathsRviz.publish(pathsToVisualize);
		pub_Paths.publish(lane_array);
	}

//	for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
//	{
//		std::ostringstream str_out;
//		str_out << UtilityHNS::UtilityH::GetHomeDirectory();
//		if(m_params.exprimentName.size() == 0)
//			str_out << UtilityHNS::DataRW::LoggingMainfolderName;
//		else
//			str_out << UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_params.exprimentName;
//
//		str_out << UtilityHNS::DataRW::GlobalPathLogFolderName;
//		str_out << "GlobalPath_";
//		str_out << i;
//		str_out << "_";
//		PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), generatedTotalPaths.at(i));
//	}
}

void GlobalPlanner::VisualizeDestinations(std::vector<PlannerHNS::WayPoint>& destinations, const int& iSelected)
{
	visualization_msgs::MarkerArray goals_array;

	for(unsigned int i=0; i< destinations.size(); i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "HMI_Destinations";
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.action = visualization_msgs::Marker::ADD;
		//marker.scale.x = 3.25;
		//marker.scale.y = 3.25;
		marker.scale.z = 3.25;
		marker.color.a = 0.9;
		marker.id = i;
		if(i == iSelected)
		{
			marker.color.r = 1;
			marker.color.g = 0;
			marker.color.b = 0;
		}
		else
		{
			marker.color.r = 0.2;
			marker.color.g = 0.8;
			marker.color.b = 0.2;
		}
		marker.pose.position.x = destinations.at(i).pos.x;
		marker.pose.position.y = destinations.at(i).pos.y;
		marker.pose.position.z = destinations.at(i).pos.z;
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(destinations.at(i).pos.a);

		std::ostringstream str_out;
		str_out << "G";
		marker.text = str_out.str();

		goals_array.markers.push_back(marker);
	}
	pub_GoalsListRviz.publish(goals_array);
}

void GlobalPlanner::SaveSimulationData()
{
	std::vector<std::string> simulationDataPoints;
	std::ostringstream startStr;
	startStr << m_StartPose.pos.x << "," << m_StartPose.pos.y << "," << m_StartPose.pos.z << "," << m_StartPose.pos.a << ","<< m_StartPose.cost << "," << 0 << ",";
	simulationDataPoints.push_back(startStr.str());

	for(unsigned int i=0; i < m_GoalsPos.size(); i++)
	{
		std::ostringstream goalStr;
		goalStr << m_GoalsPos.at(i).pos.x << "," << m_GoalsPos.at(i).pos.y << "," << m_GoalsPos.at(i).pos.z << "," << m_GoalsPos.at(i).pos.a << "," << 0 << "," << 0 << ",destination_" << i+1 << ",";
		simulationDataPoints.push_back(goalStr.str());
	}

	std::string header = "X,Y,Z,A,C,V,name,";

	std::ostringstream fileName;
	if(m_params.exprimentName.size() == 0)
		fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+ UtilityHNS::DataRW::SimulationFolderName;
	else
		fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+ UtilityHNS::DataRW::ExperimentsFolderName + m_params.exprimentName + UtilityHNS::DataRW::SimulationFolderName;

	fileName << "EgoCar.csv";
	std::ofstream f(fileName.str().c_str());

	if(f.is_open())
	{
		if(header.size() > 0)
			f << header << "\r\n";
		for(unsigned int i = 0 ; i < simulationDataPoints.size(); i++)
			f << simulationDataPoints.at(i) << "\r\n";
	}

	f.close();
}

int GlobalPlanner::LoadSimulationData()
{
	std::ostringstream fileName;
	fileName << "EgoCar.csv";

	std::string simuDataFileName = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName + fileName.str();
	if(m_params.exprimentName.size() > 1)
		simuDataFileName = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_params.exprimentName + UtilityHNS::DataRW::SimulationFolderName + fileName.str();

	UtilityHNS::SimulationFileReader sfr(simuDataFileName);
	UtilityHNS::SimulationFileReader::SimulationData data;

	int nData = sfr.ReadAllData(data);
	if(nData == 0)
		return 0;

	m_CurrentPose = PlannerHNS::WayPoint(data.startPoint.x, data.startPoint.y, data.startPoint.z, data.startPoint.a);
	m_GoalsPos.push_back(PlannerHNS::WayPoint(data.goalPoint.x, data.goalPoint.y, data.goalPoint.z, data.goalPoint.a));

	for(unsigned int i=0; i < data.simuCars.size(); i++)
	{
		m_GoalsPos.push_back(PlannerHNS::WayPoint(data.simuCars.at(i).x, data.simuCars.at(i).y, data.simuCars.at(i).z, data.simuCars.at(i).a));
	}

	return nData;
}

void GlobalPlanner::LoadDestinations(const std::string& fileName)
{
	std::string dstFileName = fileName;
	UtilityHNS::DestinationsDataFileReader destination_data(dstFileName);
	if(destination_data.ReadAllData() > 0)
	{
		m_GoalsPos.clear();
		PlannerHNS::HMI_MSG dest_msg;
		dest_msg.msg_id = m_iMessageID++;
		dest_msg.bErr = false;
		dest_msg.type = PlannerHNS::DESTINATIONS_MSG;
		if(m_iCurrentGoalIndex >= 0 && m_iCurrentGoalIndex < m_GoalsPos.size())
		{
			dest_msg.curr_destination_id = m_GoalsPos.at(m_iCurrentGoalIndex).id;
		}
		if(m_iCurrentGoalIndex+1 >= 0 && m_iCurrentGoalIndex+1 < m_GoalsPos.size())
		{
			dest_msg.next_destination_id = m_GoalsPos.at(m_iCurrentGoalIndex+1).id;
		}
		PlannerHNS::DESTINATION d;
		for(auto& x: destination_data.m_data_list)
		{
			d.id = x.id;
			d.name = x.name;
			d.hour = x.hour;
			d.minute = x.minute;
			dest_msg.destinations.push_back(d);
			PlannerHNS::WayPoint p(x.x, x.y, x.z, x.angle);
			p.pos.lon = x.lon;
			p.pos.lat = x.lat;
			p.pos.alt = x.alt;
			p.id = d.id;
			m_GoalsPos.push_back(p);
		}

		autoware_msgs::State auto_state_msg;
		auto_state_msg.mission_state = dest_msg.CreateStringMessage();
		pub_hmi_mission.publish(auto_state_msg);
		//std::cout << "Read Destinations and send message ! " << std::endl;
	}
	else
	{
		std::cout << "Failed Read Destinations and send message ! " << std::endl;
	}
}

bool GlobalPlanner::UpdateGoalIndex()
{
	if(m_bWaitingState && UtilityHNS::UtilityH::GetTimeDiffNow(m_WaitingTimer) > m_params.waitingTime)
	{
		int curr_index = m_iCurrentGoalIndex;

		if(m_params.bEnableReplanning) //infinite loop
		{
			m_iCurrentGoalIndex = (m_iCurrentGoalIndex + 1) % m_GoalsPos.size(); 
			
		}
		else //replan when new goal pose is arrived
		{
			if(m_iCurrentGoalIndex+1 <  (int)m_GoalsPos.size())
			{
				m_iCurrentGoalIndex ++;
			}
		}

		if(m_iCurrentGoalIndex != curr_index)
		{
			return true;
		}
	}

	//std::cout << "Waiting ... " << m_bWaitingState <<  UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer) << ", " << m_params.waitingTime << ", Goals: " << m_GoalsPos.size() <<   std::endl;

	return false;
}

void GlobalPlanner::MainLoop()
{
	ros::Rate loop_rate(10);
	UtilityHNS::UtilityH::GetTickCount(m_animation_timer);

	while (ros::ok())
	{
		ros::spinOnce();
		//LoadMap();
		if(!m_MapHandler.IsMapLoaded())
		{
			m_MapHandler.LoadMap(m_Map, m_params.bEnableLaneChange);

			if(m_MapHandler.IsMapLoaded())
			{
				visualization_msgs::MarkerArray map_marker_array;
				PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array, false);
				pub_MapRviz.publish(map_marker_array);
			}
		}

		if(m_bStart && m_MapHandler.IsMapLoaded() && m_GoalsPos.size() > 0)
		{
			bool bMakeNewPlan = false;
			bool bDestinationReachSend= false;

			if(!m_bWaitingState && PlannerHNS::PlanningHelpers::CheckForEndOfPaths(m_GeneratedTotalPaths, m_CurrentPose, m_params.endOfPathDistance) >= 0 && m_VehicleState.speed < 0.25)
			{
				UtilityHNS::UtilityH::GetTickCount(m_WaitingTimer);
				m_bWaitingState = true;
			}

			if(m_params.bEnableHMI)
			{
				SendAvailableOptionsHMI();
				bMakeNewPlan = UpdateGoalWithHMI();
			}
			else
			{
				bMakeNewPlan = UpdateGoalIndex();
				//std::cout << "Goal Index Updated: " << m_iCurrentGoalIndex << ", New Plan: " << bMakeNewPlan << std::endl;
			}

			if(m_iCurrentGoalIndex >= 0 && (m_bReplanSignal || bMakeNewPlan || m_GeneratedTotalPaths.size() == 0))
			{
				std::cout << "Current Goal Index = " << m_iCurrentGoalIndex << std::endl << std::endl;
				PlannerHNS::WayPoint goalPoint = m_GoalsPos.at(m_iCurrentGoalIndex);
				bool bNewPlan = GenerateGlobalPlan(m_CurrentPose, goalPoint, m_GeneratedTotalPaths);

				if(bNewPlan)
				{
					m_bWaitingState = false;
					m_bReStartState = false;
					m_bStoppingState = false;
					m_bSlowDownState = false;
					m_bReplanSignal = false;
					bMakeNewPlan = false;
					VisualizeAndSend(m_GeneratedTotalPaths);
				}
			}

			ClearOldCostFromMap();
			VisualizeDestinations(m_GoalsPos, m_iCurrentGoalIndex);
			if(m_bEnableAnimation)
			{
				AnimatedVisualizationForGlobalPath(0);
			}
		}

		loop_rate.sleep();
	}
}

PlannerHNS::ACTION_TYPE GlobalPlanner::FromMsgAction(const PlannerHNS::MSG_ACTION& msg_action)
{
	switch(msg_action)
	{
	case PlannerHNS::MSG_FORWARD_ACTION:
		return PlannerHNS::FORWARD_ACTION;
	case PlannerHNS::MSG_LEFT_TURN_ACTION:
		return PlannerHNS::LEFT_TURN_ACTION;
	case PlannerHNS::MSG_RIGHT_TURN_ACTION:
		return PlannerHNS::RIGHT_TURN_ACTION;
	case PlannerHNS::MSG_STOP_ACTION:
		return PlannerHNS::STOP_ACTION;
	case PlannerHNS::MSG_SWERVE_ACTION:
		return PlannerHNS::SWERVE_ACTION;
	case PlannerHNS::MSG_START_ACTION:
		return PlannerHNS::START_ACTION;
	case PlannerHNS::MSG_SLOWDOWN_ACTION:
		return PlannerHNS::SLOWDOWN_ACTION;
	case PlannerHNS::MSG_BACKWARD_ACTION:
		return PlannerHNS::BACKWARD_ACTION;
	case PlannerHNS::MSG_CHANGE_DESTINATION:
		return PlannerHNS::CHANGE_DESTINATION;
	case PlannerHNS::MSG_WAITING_ACTION:
		return PlannerHNS::WAITING_ACTION;
	case PlannerHNS::MSG_DESTINATION_REACHED:
		return PlannerHNS::DESTINATION_REACHED;
	default:
		return PlannerHNS::UNKOWN_ACTION;
	}
}

PlannerHNS::MSG_ACTION GlobalPlanner::ToMsgAction(const PlannerHNS::ACTION_TYPE& action)
{
	switch(action)
	{
	case PlannerHNS::FORWARD_ACTION:
		return PlannerHNS::MSG_FORWARD_ACTION;
	case PlannerHNS::LEFT_TURN_ACTION:
		return PlannerHNS::MSG_LEFT_TURN_ACTION;
	case PlannerHNS::RIGHT_TURN_ACTION:
		return PlannerHNS::MSG_RIGHT_TURN_ACTION;
	case PlannerHNS::STOP_ACTION:
		return PlannerHNS::MSG_STOP_ACTION;
	case PlannerHNS::SWERVE_ACTION:
		return PlannerHNS::MSG_SWERVE_ACTION;
	case PlannerHNS::START_ACTION:
		return PlannerHNS::MSG_START_ACTION;
	case PlannerHNS::SLOWDOWN_ACTION:
		return PlannerHNS::MSG_SLOWDOWN_ACTION;
	case PlannerHNS::BACKWARD_ACTION:
		return PlannerHNS::MSG_BACKWARD_ACTION;
	case PlannerHNS::CHANGE_DESTINATION:
		return PlannerHNS::MSG_CHANGE_DESTINATION;
	case PlannerHNS::WAITING_ACTION:
		return PlannerHNS::MSG_WAITING_ACTION;
	case PlannerHNS::DESTINATION_REACHED:
		return PlannerHNS::MSG_DESTINATION_REACHED;
	default:
		return PlannerHNS::MSG_FORWARD_ACTION;
	}
}

}
