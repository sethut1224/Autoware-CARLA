/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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

#ifndef OP_BEHAVIOR_SELECTOR_CORE
#define OP_BEHAVIOR_SELECTOR_CORE

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <autoware_msgs/LaneArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/ControlCommand.h>
#include <autoware_msgs/Waypoint.h>
#include <visualization_msgs/MarkerArray.h>
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/DecisionMaker.h"
#include "op_utility/DataRW.h"
#include "op_ros_helpers/ROSMapHandler.h"
#include "op_ros_helpers/ROSVelocityHandler.h"
#include "op_ros_helpers/op_ParamsHandler.h"

#define LOG_LOCAL_PLANNING_DATA

namespace BehaviorGeneratorNS
{

class BehaviorGen
{
protected: //Planning Related variables

	//Control Related
	int m_ControlFrequency;
	std::vector<double> dt_list;

	geometry_msgs::Pose m_OriginPos;
	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	PlannerHNS::VehicleState m_VehicleStatus;

	std::vector<PlannerHNS::WayPoint> m_temp_path;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathsToUse;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_LanesRollOutsToUse;

	PlannerHNS::RoadNetwork m_Map;

	PlannerHNS::TrajectoryCost m_TrajectoryBestCost;
	bool bBestCost;

	PlannerHNS::DecisionMaker m_BehaviorGenerator;
	PlannerHNS::BehaviorState m_CurrentBehavior;
	bool m_bRequestNewPlanSent;
	bool m_bShowActualDrivingPath;

  	std::vector<std::string> m_LogData;
  	std::vector<std::pair<PlannerHNS::WayPoint, PlannerHNS::PolygonShape> > m_ActualDrivingPath;

  	PlannerHNS::PlanningParams m_PlanningParams;
  	PlannerHNS::CAR_BASIC_INFO m_CarInfo;
  	PlannerHNS::ControllerParams m_ControlParams;

  	autoware_msgs::Lane m_CurrentTrajectoryToSend;
  	bool bNewLightStatus;
	bool bNewLightSignal;
	PlannerHNS::TRAFFIC_LIGHT_TYPE  m_CurrLightStatus;
	std::vector<PlannerHNS::TrafficLight> m_CurrTrafficLight;
	std::vector<PlannerHNS::TrafficLight> m_PrevTrafficLight;

	geometry_msgs::TwistStamped m_Twist_raw;
	geometry_msgs::TwistStamped m_Twist_cmd;
	autoware_msgs::ControlCommand m_Ctrl_cmd;

	std::string m_ExperimentFolderName;

	//timeout temp behavior, outside the behavior state
	timespec m_TimeSinceLastChange;

	//ROS messages (topics)
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_TotalLocalPath;
	ros::Publisher pub_LocalPath;
	ros::Publisher pub_LocalBasePath;
	ros::Publisher pub_ClosestIndex;
	ros::Publisher pub_BehaviorState;
	ros::Publisher pub_SimuBoxPose;
	ros::Publisher pub_SelectedPathRviz;
	ros::Publisher pub_TargetSpeedRviz;
	ros::Publisher pub_ActualSpeedRviz;
	ros::Publisher pub_DetectedLight;
	ros::Publisher pub_CurrGlobalLocalPathsIds;
	ros::Publisher pub_RequestReplan;
	ros::Publisher pub_BehaviorStateRviz;
	ros::Publisher pub_CurrDrivingPathRviz;

	// define subscribers.
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_GlobalPlannerPaths;
	ros::Subscriber sub_LocalPlannerPaths;
	ros::Subscriber sub_Trajectory_Cost;
	ros::Subscriber sub_TrafficLightStatus;
	ros::Subscriber sub_TrafficLightSignals;

	ros::Subscriber sub_twist_cmd;
	ros::Subscriber sub_twist_raw;
	ros::Subscriber sub_ctrl_cmd;

	// Control Topics Sections
	//----------------------------
	void callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetTwistCMD(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCommandCMD(const autoware_msgs::ControlCommandConstPtr& msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	//----------------------------

	//Path Planning Section
	//----------------------------
	void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr& msg);
	void CollectRollOutsByGlobalPath(std::vector< std::vector<PlannerHNS::WayPoint> >& local_rollouts);
	bool CompareTrajectoriesWithIds(std::vector<std::vector<PlannerHNS::WayPoint> >& paths, std::vector<int>& local_ids);
	//----------------------------

	//Traffic Information Section
	//----------------------------
	void callbackGetTrafficLightStatus(const autoware_msgs::TrafficLight & msg);
	void callbackGetTrafficLightSignals(const autoware_msgs::Signals& msg);
	//----------------------------

	//Helper Functions
  void UpdatePlanningParams(ros::NodeHandle& _nh);
  void SendLocalPlanningTopics();
  void VisualizeLocalPlanner();
  void LogLocalPlanningInfo(double dt);
  void InsertNewActualPathPair(const double& min_record_distance = 1.0);

public:
  BehaviorGen();
  ~BehaviorGen();
  void MainLoop();

  PlannerHNS::MapHandler m_MapHandler;
  PlannerHNS::VelocityHandler m_VelHandler;
  PlannerHNS::ParamsHandler m_ParamsHandler;
};

}

#endif  // OP_BEHAVIOR_SELECTOR_CORE
