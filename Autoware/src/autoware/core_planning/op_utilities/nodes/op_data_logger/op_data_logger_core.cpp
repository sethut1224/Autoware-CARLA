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

#include "op_data_logger_core.h"
#include "op_utility/UtilityH.h"
#include "math.h"
#include "op_planner/MatrixOperations.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/KmlMapLoader.h"
#include "op_planner/Lanelet2MapLoader.h"
#include "op_planner/VectorMapLoader.h"

namespace DataLoggerNS
{

OpenPlannerDataLogger::OpenPlannerDataLogger()
{
	bMap = false;
	m_iSimuCarsNumber = 5;
	m_bLightAndSignsLog = false;
	m_bPredictionLog = false;
	m_bControlLog = false;
	m_bSimulatedCars = false;
	m_ExperimentName = "";

	ros::NodeHandle _nh("~");
	UpdatePlanningParams(_nh);

	UtilityHNS::UtilityH::GetTickCount(m_Timer);

	sub_behavior_state 	= nh.subscribe("/op_current_behavior",	10,  &OpenPlannerDataLogger::callbackGetBehaviorState, 	this);
	sub_current_pose = nh.subscribe("/current_pose", 1,	&OpenPlannerDataLogger::callbackGetCurrentPose, this);
	sub_twist_raw = nh.subscribe("/twist_raw", 1, &OpenPlannerDataLogger::callbackGetTwistRaw, this);
	sub_twist_cmd = nh.subscribe("/twist_cmd", 1, &OpenPlannerDataLogger::callbackGetTwistCMD, this);
	//sub_ctrl_cmd = nh.subscribe("/ctrl_cmd", 1, &BehaviorGen::callbackGetCommandCMD, this);
	int bVelSource = 1;
	_nh.getParam("/op_common_params/velocitySource", bVelSource);
	if(bVelSource == 0)
		sub_robot_odom = nh.subscribe("/odometry", 1, &OpenPlannerDataLogger::callbackGetRobotOdom, this);
	else if(bVelSource == 1)
		sub_current_velocity = nh.subscribe("/current_velocity", 1, &OpenPlannerDataLogger::callbackGetVehicleStatus, this);
	else if(bVelSource == 2)
		sub_can_info = nh.subscribe("/can_info", 1, &OpenPlannerDataLogger::callbackGetCANInfo, this);

	//Prediction Section
	//----------------------------
	if(m_bPredictionLog)
	{
		sub_predicted_objects = nh.subscribe("/detection/contour_tracker/objects", 1, &OpenPlannerDataLogger::callbackGetPredictedObjects, this);
	}
	//----------------------------

	//Traffic Information Section
	//----------------------------
	if(m_bLightAndSignsLog)
	{
		sub_TrafficLightStatus = nh.subscribe("/light_color", 1, &OpenPlannerDataLogger::callbackGetTrafficLightStatus, this);
		sub_TrafficLightSignals	= nh.subscribe("/roi_signal", 1, &OpenPlannerDataLogger::callbackGetTrafficLightSignals, this);
		sub_OpTrafficLightSignal = nh.subscribe("/op_detected_light", 1, &OpenPlannerDataLogger::callbackGetOpenPlannerTrafficLightSignal, this);
	}
	//----------------------------

	//Path Planning Section
	//----------------------------
	sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &OpenPlannerDataLogger::callbackGetGlobalPlannerPath, this);
	sub_LocalPlannerPaths = nh.subscribe("/op_local_selected_trajectory", 1, &OpenPlannerDataLogger::callbackGetLocalPlannerPath, this);
	sub_Trajectory_Cost = nh.subscribe("/local_trajectory_cost", 1, &OpenPlannerDataLogger::callbackGetLocalTrajectoryCost, this);
	//----------------------------

	//Subscriptions for the simulated cars
	if(m_bSimulatedCars)
	{
		std::cout << "Logging Simulated Cars Enabled" << std::endl;
		VehicleDataContainer vc;
		vc.id = 0;
		m_SimulatedVehicle.push_back(vc);
		for(int i=1; i <= m_iSimuCarsNumber; i++)
		{
			std::ostringstream str_path_beh, str_pose;
			str_path_beh << "simu_car_path_beh_" << i;

			ros::Subscriber _sub_path_beh;
			_sub_path_beh =  nh.subscribe(str_path_beh.str(), 1, &OpenPlannerDataLogger::callbackGetSimuCarsPathAndState, this);

			sub_simu_paths.push_back(_sub_path_beh);

			str_pose << "sim_box_pose_" << i;
			ros::Subscriber _sub;
			_sub =  nh.subscribe(str_pose.str(), 1, &OpenPlannerDataLogger::callbackGetSimuPose, this);
			sub_objs.push_back(_sub);

			vc.id = i;
			m_SimulatedVehicle.push_back(vc);
			m_SimulationLogData.push_back(std::vector<std::string>());
		}
	}

	std::cout << "OpenPlannerDataLogger initialized successfully " << std::endl;
}

OpenPlannerDataLogger::~OpenPlannerDataLogger()
{
	std::ostringstream fileName;
	if(m_ExperimentName.size() == 0)
		fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName;
	else
		fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_ExperimentName;

	if(m_bSimulatedCars)
	{
		for(int i=0; i < m_iSimuCarsNumber; i++)
		{
			ostringstream car_name;
			car_name << "sim_car_no_" << i+1;
			car_name << "_";
			UtilityHNS::DataRW::WriteLogData(fileName.str()+UtilityHNS::DataRW::SimulationFolderName,
					car_name.str(),
					"time_diff,distance_diff, heading_diff, velocity_diff, rms, state_diff, actual_v, kalman_v," , m_SimulationLogData.at(i));
		}
	}

	if(m_bLightAndSignsLog)
	{
		UtilityHNS::DataRW::WriteLogData(fileName.str() + UtilityHNS::DataRW::StatesLogFolderName, "MainLog",
					"time, dt, Beh_State_i, Beh_State_str, DetectedLight, MapLight, MapLight_ID, Lights_n, Lights_IDs, Lights_Types, "
					"Selected_Traj, Indicator, Follow_Dist, Follow_Vel, Max_Vel, "
					"Velocity, Steer, X, Y, Z, Theta,"
					, m_TrafficAndSignLogData);
	}
}

void OpenPlannerDataLogger::UpdatePlanningParams(ros::NodeHandle& _nh)
{
	_nh.getParam("/op_common_params/enableSwerving", m_PlanningParams.enableSwerving);
	if(m_PlanningParams.enableSwerving)
		m_PlanningParams.enableFollowing = true;
	else
		_nh.getParam("/op_common_params/enableFollowing", m_PlanningParams.enableFollowing);

	_nh.getParam("/op_common_params/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
	_nh.getParam("/op_common_params/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);

	_nh.getParam("/op_common_params/maxVelocity", m_PlanningParams.maxSpeed);
	_nh.getParam("/op_common_params/minVelocity", m_PlanningParams.minSpeed);
	_nh.getParam("/op_common_params/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);

	_nh.getParam("/op_common_params/pathDensity", m_PlanningParams.pathDensity);

	_nh.getParam("/op_common_params/rollOutDensity", m_PlanningParams.rollOutDensity);
	if(m_PlanningParams.enableSwerving)
		_nh.getParam("/op_common_params/rollOutsNumber", m_PlanningParams.rollOutNumber);
	else
		m_PlanningParams.rollOutNumber = 0;

	_nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);

	_nh.getParam("/op_common_params/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("/op_common_params/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);

	_nh.getParam("/op_common_params/enableLaneChange", m_PlanningParams.enableLaneChange);

	_nh.getParam("/op_common_params/front_length", m_CarInfo.front_length);
	_nh.getParam("/op_common_params/back_length", m_CarInfo.back_length);
	_nh.getParam("/op_common_params/height", m_CarInfo.height);
	_nh.getParam("/op_common_params/width", m_CarInfo.width);
	_nh.getParam("/op_common_params/length", m_CarInfo.length);
	_nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
	_nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
	_nh.getParam("/op_common_params/maxWheelAngle", m_CarInfo.max_wheel_angle);
	_nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
	_nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);
	m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
	m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;

	PlannerHNS::ControllerParams controlParams;
	controlParams.Steering_Gain = PlannerHNS::PID_CONST(0.07, 0.02, 0.01);
	controlParams.Velocity_Gain = PlannerHNS::PID_CONST(0.1, 0.005, 0.1);
	nh.getParam("/op_common_params/steeringDelay", controlParams.SteeringDelay);
	nh.getParam("/op_common_params/minPursuiteDistance", controlParams.minPursuiteDistance );
	nh.getParam("/op_common_params/additionalBrakingDistance", m_PlanningParams.additionalBrakingDistance );
	nh.getParam("/op_common_params/giveUpDistance", m_PlanningParams.giveUpDistance );

	int iSource = 0;
	_nh.getParam("/op_common_params/mapSource" , iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_AUTOWARE;
	else if (iSource == 1)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_MapType = PlannerHNS::MAP_KML_FILE;
	else if(iSource == 3)
	{
		m_MapType = PlannerHNS::MAP_LANELET_2;
		std::string str_origin;
		nh.getParam("/op_common_params/lanelet2_origin" , str_origin);
		std::vector<std::string> lat_lon_alt = PlannerHNS::MappingHelpers::SplitString(str_origin, ",");
		if(lat_lon_alt.size() == 3)
		{
			m_Map.origin.pos.lat = atof(lat_lon_alt.at(0).c_str());
			m_Map.origin.pos.lon = atof(lat_lon_alt.at(1).c_str());
			m_Map.origin.pos.alt = atof(lat_lon_alt.at(2).c_str());
		}
	}

	_nh.getParam("/op_common_params/mapFileName" , m_MapPath);
	_nh.getParam("/op_data_logger/experimentName" , m_ExperimentName);
	_nh.getParam("/op_behavior_selector/evidence_trust_number", m_PlanningParams.nReliableCount);


	if(m_ExperimentName.size() > 0)
	{
		if(m_ExperimentName.at(m_ExperimentName.size()-1) != '/')
			m_ExperimentName.push_back('/');
	}

	UtilityHNS::DataRW::CreateLoggingMainFolder();
	if(m_ExperimentName.size() > 1)
	{
		UtilityHNS::DataRW::CreateExperimentFolder(m_ExperimentName);
	}

	_nh.getParam("/op_data_logger/lightsAndSignsLog", m_bLightAndSignsLog);
	_nh.getParam("/op_data_logger/predictionLog", m_bPredictionLog);
	_nh.getParam("/op_data_logger/controlLog", m_bControlLog);
	_nh.getParam("/op_data_logger/simulatedCars", m_bSimulatedCars);
}

//Simulated Vehicles Section
//----------------------------
void OpenPlannerDataLogger::callbackGetSimuPose(const geometry_msgs::PoseArray& msg)
{
	for(unsigned int i=0; i < m_SimulatedVehicle.size(); i++)
	{
		if(msg.poses.size() > 3 )
		{
			if(m_SimulatedVehicle.at(i).id == msg.poses.at(0).position.x)
			{
				//std::cout << "Receive SimuPOse Data ... " << msg.poses.at(0).position.x << ", " << msg.header.stamp <<  std::endl;

				m_SimulatedVehicle.at(i).pose.v = msg.poses.at(0).position.y;
				m_SimulatedVehicle.at(i).pose.pos.x = msg.poses.at(1).position.x;
				m_SimulatedVehicle.at(i).pose.pos.y = msg.poses.at(1).position.y;
				m_SimulatedVehicle.at(i).pose.pos.z = msg.poses.at(1).position.z;
				m_SimulatedVehicle.at(i).pose.pos.a = tf::getYaw(msg.poses.at(1).orientation);

				if(msg.poses.at(3).orientation.w == 0)
					m_SimulatedVehicle.at(i).beh.indicator = PlannerHNS::INDICATOR_LEFT;
				else if(msg.poses.at(3).orientation.w == 1)
					m_SimulatedVehicle.at(i).beh.indicator = PlannerHNS::INDICATOR_RIGHT;
				else if(msg.poses.at(3).orientation.w == 2)
					m_SimulatedVehicle.at(i).beh.indicator = PlannerHNS::INDICATOR_BOTH;
				else if(msg.poses.at(3).orientation.w == 3)
					m_SimulatedVehicle.at(i).beh.indicator = PlannerHNS::INDICATOR_NONE;

				m_SimulatedVehicle.at(i).pose_time = msg.header.stamp;

				break;
			}
		}
	}
}

void OpenPlannerDataLogger::callbackGetSimuCarsPathAndState(const autoware_msgs::LaneConstPtr& msg )
{
	//std::cout << "Receive Lane Data ... " << std::endl;

	for(unsigned int i=0; i < m_SimulatedVehicle.size(); i++)
	{
		if(m_SimulatedVehicle.at(i).id == msg->lane_id)
		{
			m_SimulatedVehicle.at(i).path.clear();
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(*msg, m_SimulatedVehicle.at(i).path);
			m_SimulatedVehicle.at(i).beh.state = GetStateFromNumber(msg->lane_index);
			m_SimulatedVehicle.at(i).path_time = msg->header.stamp;
			break;
		}
	}
}
//----------------------------

//Prediction Section
//----------------------------
void OpenPlannerDataLogger::callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	m_PredictedObjects.clear();
	PlannerHNS::DetectedObject obj;

	//std::cout << "Receive Prediction Data From Ego Vehicle... " << msg->header.stamp <<  std::endl;
	m_pred_time = msg->header.stamp;

	for(unsigned int i = 0 ; i <msg->objects.size(); i++)
	{
		if(msg->objects.at(i).id > 0)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(msg->objects.at(i), obj);


			obj.behavior_state = GetBehStateFromNumber(msg->objects.at(i).behavior_state);
			int i_best_trajectory = -1;
			double max_cost = 0;
			for(unsigned int k=0; k < obj.predTrajectories.size(); k++)
			{
				if(obj.predTrajectories.at(k).size() > 0)
				{
					//std::cout << "Fins Max Traj Cost .............. " << obj.predTrajectories.at(k).at(0).collisionCost  << std::endl;

					if(obj.predTrajectories.at(k).at(0).collisionCost == 1)
					{

						max_cost = obj.predTrajectories.at(k).at(0).collisionCost;
						i_best_trajectory = k;
					}
				}
			}

			if(i_best_trajectory >= 0 &&  i_best_trajectory < obj.predTrajectories.size())
			{
				std::vector<PlannerHNS::WayPoint> pred_traj = obj.predTrajectories.at(i_best_trajectory);
				obj.predTrajectories.clear();
				obj.predTrajectories.push_back(pred_traj);
			}
			else
				obj.predTrajectories.clear();

			m_PredictedObjects.push_back(obj);
		}
	}
}
//----------------------------

//Functions related to Ego Vehicle Data
//----------------------------
void OpenPlannerDataLogger::callbackGetBehaviorState(const autoware_msgs::WaypointConstPtr& msg)
{
	m_CurrentBehavior = PlannerHNS::ROSHelpers::ConvertAutowareWaypointToBehaviorState(*msg);
//	std::cout << "Receive Behavior Data From Ego Vehicle... " << msg->header.stamp <<  std::endl;
}

void OpenPlannerDataLogger::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
}

void OpenPlannerDataLogger::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;

	if(fabs(m_CurrentPos.v) > 0.1)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/m_CurrentPos.v);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
}

//----------------------------

// Control Topics Sections
//----------------------------
void OpenPlannerDataLogger::callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_Twist_raw = *msg;
}

void OpenPlannerDataLogger::callbackGetTwistCMD(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_Twist_cmd = *msg;
}

void OpenPlannerDataLogger::callbackGetCommandCMD(const autoware_msgs::ControlCommandConstPtr& msg)
{
	m_Ctrl_cmd = *msg;
}

void OpenPlannerDataLogger::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{

	m_VehicleStatus.speed = msg->speed/3.6;
	m_CurrentPos.v = m_VehicleStatus.speed;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
}

void OpenPlannerDataLogger::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed ;
	if(msg->twist.twist.linear.x != 0)
		m_VehicleStatus.steer += atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
}
//----------------------------

//Traffic Information Section
//----------------------------
void OpenPlannerDataLogger::callbackGetTrafficLightStatus(const autoware_msgs::TrafficLight& msg)
{
	//std::cout << "Received Traffic Light Status : " << msg.traffic_light << std::endl;
	if(msg.traffic_light == 1) // green
		m_CurrLightStatus = PlannerHNS::GREEN_LIGHT;
	else //0 => RED , 2 => Unknown
		m_CurrLightStatus = PlannerHNS::RED_LIGHT;
}

void OpenPlannerDataLogger::callbackGetTrafficLightSignals(const autoware_msgs::Signals& msg)
{
	std::vector<PlannerHNS::TrafficLight> simulatedLights;
	std::ostringstream dataLineIds;
	std::ostringstream dataLineTypes;
	for(unsigned int i = 0 ; i < msg.Signals.size() ; i++)
	{
		PlannerHNS::TrafficLight tl;
		tl.id = msg.Signals.at(i).signalId;
		dataLineIds << tl.id << " | ";

		for(unsigned int k = 0; k < m_Map.trafficLights.size(); k++)
		{
			if(m_Map.trafficLights.at(k).id == tl.id)
			{
				tl.pose = m_Map.trafficLights.at(k).pose;
				break;
			}
		}

		switch(msg.Signals.at(i).type)
		{
		case 1:
			tl.lightType = PlannerHNS::RED_LIGHT;
			dataLineTypes << "R";
			break;
		case 2:
			tl.lightType = PlannerHNS::GREEN_LIGHT;
			dataLineTypes << "G";
			break;
		case 3:
			tl.lightType = PlannerHNS::YELLOW_LIGHT; //r = g = 1
			dataLineTypes << "Y";
			break;
		case 4:
			tl.lightType = PlannerHNS::CROSS_RED;
			dataLineTypes << "CR";
			break;
		case 5:
			tl.lightType = PlannerHNS::CROSS_GREEN;
			dataLineTypes << "CG";
			break;
		default:
			tl.lightType = PlannerHNS::UNKNOWN_LIGHT;
			dataLineTypes << "UN";
			break;
		}

		dataLineTypes << " | ";
		simulatedLights.push_back(tl);
	}
	m_CurrTrafficLightIds = dataLineIds.str();
	m_CurrTrafficLightTypes = dataLineTypes.str();
	//std::cout << "Received Traffic Lights : " << lights.markers.size() << std::endl;
	m_CurrTrafficLight = simulatedLights;
}

void OpenPlannerDataLogger::callbackGetOpenPlannerTrafficLightSignal(const autoware_msgs::ExtractedPosition& msg)
{
	std::ostringstream dataLineTypes;
	m_OpenPlannerDetectedLight.id = msg.signalId;


	switch(msg.type)
	{
	case 1:
		m_OpenPlannerDetectedLight.lightType = PlannerHNS::RED_LIGHT;
		dataLineTypes << "R";
		break;
	case 2:
		m_OpenPlannerDetectedLight.lightType = PlannerHNS::GREEN_LIGHT;
		dataLineTypes << "G";
		break;
	case 3:
		m_OpenPlannerDetectedLight.lightType = PlannerHNS::YELLOW_LIGHT; //r = g = 1
		dataLineTypes << "Y";
		break;
	case 4:
		m_OpenPlannerDetectedLight.lightType = PlannerHNS::CROSS_RED;
		dataLineTypes << "CR";
		break;
	case 5:
		m_OpenPlannerDetectedLight.lightType = PlannerHNS::CROSS_GREEN;
		dataLineTypes << "CG";
		break;
	default:
		m_OpenPlannerDetectedLight.lightType = PlannerHNS::UNKNOWN_LIGHT;
		dataLineTypes << "UN";
		break;
	}

	m_OpenPlannerLightType = dataLineTypes.str();
}
//----------------------------

//Path Planning Section
//----------------------------
void OpenPlannerDataLogger::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		bool bOldGlobalPath = m_GlobalPaths.size() == msg->lanes.size();
		m_GlobalPaths.clear();

		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);

			if(bMap)
			{
				PlannerHNS::Lane* pPrevValid = 0;
				for(unsigned int j = 0 ; j < m_temp_path.size(); j++)
				{
					PlannerHNS::Lane* pLane = 0;
					pLane = PlannerHNS::MappingHelpers::GetLaneById(m_temp_path.at(j).laneId, m_Map);
					if(!pLane)
					{
						pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMap(m_temp_path.at(j), m_Map, 1, true);

						if(!pLane && !pPrevValid)
						{
							ROS_ERROR("Map inconsistency between Global Path and Local Planer Map, Can't identify current lane.");
							return;
						}

						if(!pLane)
							m_temp_path.at(j).pLane = pPrevValid;
						else
						{
							m_temp_path.at(j).pLane = pLane;
							pPrevValid = pLane ;
						}

						m_temp_path.at(j).laneId = m_temp_path.at(j).pLane->id;
					}
					else
						m_temp_path.at(j).pLane = pLane;

					//std::cout << "StopLineInGlobalPath: " << m_temp_path.at(j).stopLineID << std::endl;
				}
			}

			m_GlobalPaths.push_back(m_temp_path);

			if(bOldGlobalPath)
			{
				bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(m_temp_path, m_GlobalPaths.at(i));
			}
		}

		if(!bOldGlobalPath)
		{
			bWayGlobalPath = true;
			for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
			{
				PlannerHNS::PlanningHelpers::FixPathDensity(m_GlobalPaths.at(i), m_PlanningParams.pathDensity);
				PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_temp_path);
				PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.35, 0.4, 0.05);
				PlannerHNS::PlanningHelpers::GenerateRecommendedSpeed(m_GlobalPaths.at(i), m_CarInfo.max_speed_forward, m_PlanningParams.speedProfileFactor);

				std::ostringstream str_out;
				str_out << UtilityHNS::UtilityH::GetHomeDirectory();
				if(m_ExperimentName.size() == 0)
					str_out << UtilityHNS::DataRW::LoggingMainfolderName;
				else
					str_out << UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_ExperimentName;

				str_out << UtilityHNS::DataRW::GlobalPathLogFolderName;
				str_out << "GlobalPath_";
				str_out << i;
				str_out << "_";
				PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), m_GlobalPaths.at(i));

			}

			std::cout << "Received New Global Path Selector! " << std::endl;
		}
		else
		{
			m_GlobalPaths.clear();
		}
	}
}

void OpenPlannerDataLogger::callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr& msg)
{
	m_TrajectoryBestCost.bBlocked = msg->is_blocked;
	m_TrajectoryBestCost.index = msg->lane_index;
	m_TrajectoryBestCost.cost = msg->cost;
	m_TrajectoryBestCost.closest_obj_distance = msg->closest_object_distance;
	m_TrajectoryBestCost.closest_obj_velocity = msg->closest_object_velocity;
}

void OpenPlannerDataLogger::callbackGetLocalPlannerPath(const autoware_msgs::LaneConstPtr& msg)
{
	PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(*msg, m_SelectedPath);
	std::ostringstream str_out;
	str_out << UtilityHNS::UtilityH::GetHomeDirectory();
	if(m_ExperimentName.size() == 0)
		str_out << UtilityHNS::DataRW::LoggingMainfolderName;
	else
		str_out << UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_ExperimentName;

	str_out << UtilityHNS::DataRW::PathLogFolderName;
	str_out << "SelectedPath_";
	PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), m_SelectedPath);
}
//----------------------------

void OpenPlannerDataLogger::MainLoop()
{
	timespec planningTimer;
	UtilityHNS::UtilityH::GetTickCount(planningTimer);
	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		ros::spinOnce();

		double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(planningTimer);
		UtilityHNS::UtilityH::GetTickCount(planningTimer);

		if(m_MapType == PlannerHNS::MAP_KML_FILE && !bMap)
		{
			bMap = true;
			PlannerHNS::KmlMapLoader kml_loader;
			kml_loader.LoadKML(m_MapPath, m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_FOLDER && !bMap)
		{
			bMap = true;
			PlannerHNS::VectorMapLoader vec_loader;
			vec_loader.LoadFromFile(m_MapPath, m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_LANELET_2 && !bMap)
		{
			bMap = true;
			PlannerHNS::Lanelet2MapLoader map_loader;
			map_loader.LoadMap(m_MapPath, m_Map);
		}

		if(m_bLightAndSignsLog)
		{
			LogLocalTrafficInfo(dt);
		}

		if(m_bSimulatedCars)
		{
			ros::Time t;
			for(unsigned int i=0; i < m_SimulatedVehicle.size(); i++)
			{
				//if(m_SimulatedVehicle.at(i).pose_time != t && m_SimulatedVehicle.at(i).path_time != t)
				{
					for(unsigned int j = 0; j < m_PredictedObjects.size(); j++)
					{
						//std::cout << "CarID: " <<  m_SimulatedVehicle.at(i).id << ", PredCarID: " << m_PredictedObjects.at(j).id << std::endl;
						if(m_SimulatedVehicle.at(i).id == m_PredictedObjects.at(j).id)
						{
							CompareAndLog(m_SimulatedVehicle.at(i), m_PredictedObjects.at(j));
						}
					}

					//std::cout << "CarID: " <<  m_SimulatedVehicle.at(i).id << ", PoseTime: " << m_SimulatedVehicle.at(i).pose_time.toSec() << ", PredTime: " << m_pred_time.toSec() << std::endl;
					//std::cout << std::endl;
				}
			}
		}

		loop_rate.sleep();
	}
}

void OpenPlannerDataLogger::LogLocalTrafficInfo(double dt)
{
	timespec log_t;
	UtilityHNS::UtilityH::GetTickCount(log_t);
	std::ostringstream dataLine;
	dataLine << UtilityHNS::UtilityH::GetLongTime(log_t) <<"," << dt << "," <<
					m_CurrentBehavior.state << ","<<
					PlannerHNS::ROSHelpers::GetBehaviorNameFromCode(m_CurrentBehavior.state) << "," <<
					PlannerHNS::MappingHelpers::FromLightTypeToText(m_CurrLightStatus) << "," <<
					m_OpenPlannerLightType << "," <<
					m_OpenPlannerDetectedLight.id << "," <<
					m_CurrTrafficLight.size() << "," <<
					m_CurrTrafficLightIds << "," <<
					m_CurrTrafficLightTypes << "," <<
					m_CurrentBehavior.iTrajectory << "," <<
					m_CurrentBehavior.indicator << "," <<
					m_CurrentBehavior.followDistance << "," <<
					m_CurrentBehavior.followVelocity << "," <<
					m_CurrentBehavior.maxVelocity << "," <<
					m_VehicleStatus.speed << "," <<
					m_VehicleStatus.steer << "," <<
					m_CurrentPos.pos.x << "," <<
					m_CurrentPos.pos.y << "," <<
					m_CurrentPos.pos.z << "," <<
					UtilityHNS::UtilityH::SplitPositiveAngle(m_CurrentPos.pos.a)+M_PI << ",";
	if(m_TrafficAndSignLogData.size() < 150000) //in case I forget to turn off this node .. could fill the hard drive
	{
		m_TrafficAndSignLogData.push_back(dataLine.str());
	}
}

//Helper Functions
//----------------------------
//PlannerHNS::BehaviorState OpenPlannerDataLogger::ConvertBehaviorStateFromAutowareToPlannerH(const geometry_msgs::TwistStampedConstPtr& msg)
//{
//	PlannerHNS::BehaviorState behavior;
//	behavior.bNewPlan = msg->twist.linear.x;
//	behavior.followDistance = msg->twist.linear.y;
//	behavior.followVelocity = msg->twist.linear.z;
//
//	if(msg->twist.angular.x == PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT)
//		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT;
//	else if(msg->twist.angular.x == PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT)
//		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT;
//	else if(msg->twist.angular.x == PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH)
//		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH;
//	else if(msg->twist.angular.x == PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE)
//		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE;
//
//	behavior.state = GetStateFromNumber(msg->twist.angular.y);
//	behavior.iTrajectory = msg->twist.angular.z;
//
//	return behavior;
//
//}

PlannerHNS::STATE_TYPE OpenPlannerDataLogger::GetStateFromNumber(const int& iBehState)
{
	PlannerHNS::STATE_TYPE _state;
	if(iBehState == PlannerHNS::INITIAL_STATE)
		_state = PlannerHNS::INITIAL_STATE;
	else if(iBehState == PlannerHNS::WAITING_STATE)
		_state = PlannerHNS::WAITING_STATE;
	else if(iBehState == PlannerHNS::FORWARD_STATE)
		_state = PlannerHNS::FORWARD_STATE;
	else if(iBehState == PlannerHNS::STOPPING_STATE)
		_state = PlannerHNS::STOPPING_STATE;
	else if(iBehState == PlannerHNS::EMERGENCY_STATE)
		_state = PlannerHNS::EMERGENCY_STATE;
	else if(iBehState == PlannerHNS::TRAFFIC_LIGHT_STOP_STATE)
		_state = PlannerHNS::TRAFFIC_LIGHT_STOP_STATE;
	else if(iBehState == PlannerHNS::STOP_SIGN_STOP_STATE)
		_state = PlannerHNS::STOP_SIGN_STOP_STATE;
	else if(iBehState == PlannerHNS::STOP_SIGN_WAIT_STATE)
		_state = PlannerHNS::STOP_SIGN_WAIT_STATE;
	else if(iBehState == PlannerHNS::FOLLOW_STATE)
		_state = PlannerHNS::FOLLOW_STATE;
	else if(iBehState == PlannerHNS::LANE_CHANGE_STATE)
		_state = PlannerHNS::LANE_CHANGE_STATE;
	else if(iBehState == PlannerHNS::OBSTACLE_AVOIDANCE_STATE)
		_state = PlannerHNS::OBSTACLE_AVOIDANCE_STATE;
	else if(iBehState == PlannerHNS::FINISH_STATE)
		_state = PlannerHNS::FINISH_STATE;

	return _state;
}

PlannerHNS::BEH_STATE_TYPE OpenPlannerDataLogger::GetBehStateFromNumber(const int& iBehState)
{
	if(iBehState == 0)
		return PlannerHNS::BEH_FORWARD_STATE;
	else if(iBehState == 1)
		return PlannerHNS::BEH_STOPPING_STATE;
	else if(iBehState == 2)
		return PlannerHNS::BEH_BRANCH_LEFT_STATE;
	else if(iBehState == 3)
		return PlannerHNS::BEH_BRANCH_RIGHT_STATE;
	else if(iBehState == 4)
		return PlannerHNS::BEH_YIELDING_STATE;
	else if(iBehState == 5)
		return PlannerHNS::BEH_ACCELERATING_STATE;
	else if(iBehState == 6)
		return PlannerHNS::BEH_PARKING_STATE;
	else
		return PlannerHNS::BEH_FORWARD_STATE;
}

void OpenPlannerDataLogger::CompareAndLog(VehicleDataContainer& ground_truth, PlannerHNS::DetectedObject& predicted)
{
	//difference between actual and prediction
	//time diff
	//distance , orientation, velocity  diff
	//RMS diff (make both same density first) , -1 if no prediction
	//behavior state matching (1 if match , zero if not)

	double t_diff = fabs(ground_truth.path_time.toSec() - m_pred_time.toSec());
	double d_diff = hypot(ground_truth.pose.pos.y - predicted.center.pos.y, ground_truth.pose.pos.x - predicted.center.pos.x);
	double a_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(ground_truth.pose.pos.a, predicted.center.pos.a);
	double v_diff = fabs(ground_truth.pose.v - predicted.center.v);
	double rms = -1;
	int beh_state_diff = -1;

	if(predicted.predTrajectories.size() > 0)
	{
		rms = CalculateRMS(ground_truth.path, predicted.predTrajectories.at(0));
	}

	if(ground_truth.beh.state == predicted.behavior_state)
		beh_state_diff = 1;
	else
		beh_state_diff = 0;

	beh_state_diff = predicted.behavior_state;


	//"time_diff,distance_diff, heading_diff, velocity_diff, rms, state_diff, actual_v, kalman_v,"
	std::ostringstream dataLine;
	dataLine << t_diff << "," << d_diff << "," <<  a_diff << "," << v_diff << "," << rms << "," << beh_state_diff << "," << ground_truth.pose.v << "," << predicted.center.v << ",";
	m_SimulationLogData.at(ground_truth.id -1).push_back(dataLine.str());

	//std::cout << "Predicted Behavior: " << predicted.behavior_state << std::endl;
}

double OpenPlannerDataLogger::CalculateRMS(std::vector<PlannerHNS::WayPoint>& path1, std::vector<PlannerHNS::WayPoint>& path2)
{
	if(path1.size() == 0 || path2.size() == 0) return -1;

	//PlannerHNS::PlanningHelpers::FixPathDensity(path1, 0.1);
	//PlannerHNS::PlanningHelpers::FixPathDensity(path2, 0.1);

	int min_size = path1.size();
	if(path2.size() < min_size)
		min_size = path2.size();

	double rms_sum = 0;
	PlannerHNS::RelativeInfo info;
	for(unsigned int i=0; i < min_size - 1 ; i++)
	{
		PlannerHNS::PlanningHelpers::GetRelativeInfo(path1, path2.at(i), info);
		rms_sum += fabs(info.perp_distance);
		//rms_sum += hypot(path1.at(i).pos.y - path2.at(i).pos.y, path1.at(i).pos.x - path2.at(i).pos.x);
	}

	return rms_sum / (double)min_size;
}

}
