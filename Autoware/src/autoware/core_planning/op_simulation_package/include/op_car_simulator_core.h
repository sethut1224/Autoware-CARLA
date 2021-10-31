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

#ifndef OP_CAR_SIMULATOR
#define OP_CAR_SIMULATOR

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
#include "autoware_msgs/Signals.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "sensor_msgs/Joy.h"

#include "op_simu/TrajectoryFollower.h"
#include "op_planner/LocalPlannerH.h"
#include "op_planner/PlannerH.h"
#include "op_planner/MappingHelpers.h"

#include "op_simu/SimpleTracker.h"
#include "op_planner/SimuDecisionMaker.h"
#include "op_utility/DataRW.h"

#include "op_ros_helpers/ROSMapHandler.h"


namespace CarSimulatorNS
{

#define STEERING_AXIS 0
#define ACCELERATION_AXIS 1
#define BRAKE_AXIS 2
#define BUTTON_INDEX 0
#define START_BUTTON_VALUE 512

//enum MAP_SOURCE_TYPE{MAP_AUTOWARE, MAP_FOLDER, MAP_KML_FILE};

class SimuCommandParams
{
public:
	int id;
	std::string 	strID;
	std::string 	meshPath;
	std::string 	logPath;
	bool			bRvizPositions;
	bool 			bLooper;
	PlannerHNS::WayPoint startPose;
	PlannerHNS::WayPoint goalPose;
	std_msgs::ColorRGBA modelColor;
	bool			bEnableLogs;

	SimuCommandParams()
	{
		id = 1;
		bEnableLogs = true;
		bLooper = false;
		bRvizPositions = true;
		modelColor.a = 1;
		modelColor.b = 1;
		modelColor.r = 1;
		modelColor.g = 1;
	}
};

class OpenPlannerCarSimulator
{
protected:
	timespec m_PlanningTimer;
	geometry_msgs::Pose m_OriginPos;


	std::string m_BaseLinkFrameID;
	std::string m_VelodyneFrameID;

	bool m_bStepByStep;
	//bool m_bSimulatedVelodyne;
	bool m_bGoNextStep;
	PlannerHNS::RoadNetwork		m_Map;
	PlannerHNS::PlannerH		m_GlobalPlanner;
	PlannerHNS::SimuDecisionMaker* 	m_LocalPlanner;
	SimulationNS::TrajectoryFollower m_PredControl;
	std::vector<PlannerHNS::DetectedObject> m_PredictedObjects;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	PlannerHNS::VehicleState  m_JoyDesiredStatus;
	bool bPredictedObjects;

	bool bInitPos;
	bool bGoalPos;
	bool bUseWheelController;
	bool bNewLightSignal;
	std::vector<PlannerHNS::TrafficLight> m_CurrTrafficLight;
	std::vector<PlannerHNS::TrafficLight> m_PrevTrafficLight;


	SimuCommandParams m_SimParams;
	PlannerHNS::CAR_BASIC_INFO m_CarInfo;
	PlannerHNS::ControllerParams m_ControlParams;
	PlannerHNS::PlanningParams m_PlanningParams;
	PlannerHNS::BehaviorState m_CurrBehavior;

	ros::NodeHandle nh;

	tf::TransformListener m_Listener;

	ros::Publisher pub_SafetyBorderRviz;
	ros::Publisher pub_SimuBoxPose;
	ros::Publisher pub_CurrPoseRviz;
	ros::Publisher pub_LocalTrajectoriesRviz;
	ros::Publisher pub_GlobalTrajectoriesRviz;
	ros::Publisher pub_BehaviorStateRviz;
	ros::Publisher pub_PointerBehaviorStateRviz;
	ros::Publisher pub_InternalInfoRviz;
	//ros::Publisher pub_SimulatedVelodyne;
	ros::Publisher pub_CurrentLocalPath;

	// define subscribers.
	ros::Subscriber sub_initialpose;
	ros::Subscriber sub_goalpose;
	ros::Subscriber sub_predicted_objects;
	ros::Subscriber sub_TrafficLightSignals	;
	ros::Subscriber sub_StepSignal;
	//ros::Subscriber sub_cloud_clusters;
	ros::Subscriber sub_joystick;

	// Callback function for subscriber.
	void callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
	void callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg);
	void callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg);
	void callbackGetTrafficLightSignals(const autoware_msgs::Signals& msg);
	void callbackGetStepForwardSignals(const geometry_msgs::TwistStampedConstPtr& msg);
	//void callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr& msg);
	void callbackGetJoyStickInfo(const sensor_msgs::JoyConstPtr& msg);

public:
	OpenPlannerCarSimulator();

	virtual ~OpenPlannerCarSimulator();

	void MainLoop();

	void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);

  void ReadParamFromLaunchFile(PlannerHNS::CAR_BASIC_INFO& m_CarInfo,
		  PlannerHNS::ControllerParams& m_ControlParams);

  void displayFollowingInfo(const std::vector<PlannerHNS::GPSPoint>& safety_rect, PlannerHNS::WayPoint& curr_pose);
  void visualizePath(const std::vector<PlannerHNS::WayPoint>& path, ros::Publisher& pub_Rviz);

  void visualizeBehaviors();

  void SaveSimulationData();
  int LoadSimulationData(PlannerHNS::WayPoint& start_p, PlannerHNS::WayPoint& goal_p);
  void InitializeSimuCar(PlannerHNS::WayPoint start_pose);
  void PublishSpecialTF(const PlannerHNS::WayPoint& pose);


	//Mapping Section
  PlannerHNS::MapHandler m_MapHandler;

};

}

#endif  // OP_CAR_SIMULATOR
