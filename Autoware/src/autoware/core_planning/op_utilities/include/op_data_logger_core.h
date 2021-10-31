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

#ifndef OP_DATALOGGER
#define OP_DATALOGGER


#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/ExtractedPosition.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/Lane.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/ControlCommand.h>
#include <autoware_can_msgs/CANInfo.h>
#include <geometry_msgs/PoseArray.h>
#include "op_planner/RoadNetwork.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlannerCommonDef.h"


namespace DataLoggerNS
{

class VehicleDataContainer
{
public:
	int id;
	std::vector<PlannerHNS::WayPoint> path;
	PlannerHNS::BehaviorState beh;
	PlannerHNS::WayPoint pose;
	ros::Time path_time;
	ros::Time pose_time;
};

class OpenPlannerDataLogger
{

protected:
	ros::NodeHandle nh;
	timespec m_Timer;

	std::vector<VehicleDataContainer>  m_SimulatedVehicle;
	std::vector<PlannerHNS::DetectedObject> m_PredictedObjects;
	ros::Time m_pred_time;
	PlannerHNS::BehaviorState m_CurrentBehavior;
	PlannerHNS::WayPoint m_CurrentPos;
	PlannerHNS::VehicleState m_VehicleStatus;

	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;
	PlannerHNS::RoadNetwork m_Map;
	bool bMap;
	int m_iSimuCarsNumber;
	bool m_bLightAndSignsLog;
	bool m_bPredictionLog;
	bool m_bControlLog;
	bool m_bSimulatedCars;
	std::string m_ExperimentName;

	std::vector<std::vector<std::string> >  m_SimulationLogData;
	std::vector<std::string> m_TrafficAndSignLogData;

	PlannerHNS::PlanningParams m_PlanningParams;
	PlannerHNS::CAR_BASIC_INFO m_CarInfo;
	PlannerHNS::TRAFFIC_LIGHT_TYPE  m_CurrLightStatus;
	std::vector<PlannerHNS::TrafficLight> m_CurrTrafficLight;
	PlannerHNS::TrafficLight m_OpenPlannerDetectedLight;
	std::string m_CurrTrafficLightIds;
	std::string m_CurrTrafficLightTypes;
	std::string m_OpenPlannerLightType;
	PlannerHNS::TrajectoryCost m_TrajectoryBestCost;
	std::vector<PlannerHNS::WayPoint> m_temp_path;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	std::vector<PlannerHNS::WayPoint> m_SelectedPath;
	geometry_msgs::TwistStamped m_Twist_raw;
	geometry_msgs::TwistStamped m_Twist_cmd;
	autoware_msgs::ControlCommand m_Ctrl_cmd;


	bool bWayGlobalPath;

	void UpdatePlanningParams(ros::NodeHandle& _nh);

	ros::Subscriber sub_predicted_objects;
	ros::Subscriber sub_behavior_state;
	std::vector<ros::Subscriber> sub_simu_paths;
	std::vector<ros::Subscriber> sub_objs;
	ros::Subscriber sub_TrafficLightStatus;
	ros::Subscriber sub_TrafficLightSignals;
	ros::Subscriber sub_OpTrafficLightSignal;
	ros::Subscriber sub_GlobalPlannerPaths;
	ros::Subscriber sub_LocalPlannerPaths;
	ros::Subscriber sub_Trajectory_Cost;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_twist_raw;
	ros::Subscriber sub_twist_cmd;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_can_info;

	// Control Topics Sections
	//----------------------------
	void callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetTwistCMD(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCommandCMD(const autoware_msgs::ControlCommandConstPtr& msg);
	void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	//----------------------------

	//Simulated Vehicles Section
	//----------------------------
	void callbackGetSimuPose(const geometry_msgs::PoseArray &msg);
	void callbackGetSimuCarsPathAndState(const autoware_msgs::LaneConstPtr& msg);
	//----------------------------

	//Prediction Section
	//----------------------------
	void callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg);
	//----------------------------

	//General Section
	//--------------------------
	void callbackGetBehaviorState(const autoware_msgs::WaypointConstPtr& msg );
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	//--------------------------

	//Traffic Information Section
	//----------------------------
	void callbackGetTrafficLightStatus(const autoware_msgs::TrafficLight & msg);
	void callbackGetTrafficLightSignals(const autoware_msgs::Signals& msg);
	void callbackGetOpenPlannerTrafficLightSignal(const autoware_msgs::ExtractedPosition& msg);

	//----------------------------

	//Path Planning Section
	//----------------------------
	void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalPlannerPath(const autoware_msgs::LaneConstPtr& msg);
	void callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr& msg);
	//----------------------------

public:
	OpenPlannerDataLogger();
	virtual ~OpenPlannerDataLogger();
	void MainLoop();

private:
	void LogLocalTrafficInfo(double dt);
	//Helper Functions
	//PlannerHNS::BehaviorState ConvertBehaviorStateFromAutowareToPlannerH(const geometry_msgs::TwistStampedConstPtr& msg);
	PlannerHNS::STATE_TYPE GetStateFromNumber(const int& iBehState);
	PlannerHNS::BEH_STATE_TYPE GetBehStateFromNumber(const int& iBehState);
	void CompareAndLog(VehicleDataContainer& ground_truth, PlannerHNS::DetectedObject& predicted);
	double CalculateRMS(std::vector<PlannerHNS::WayPoint>& path1, std::vector<PlannerHNS::WayPoint>& path2);
};

}

#endif  // OP_DATALOGGER
