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

#ifndef OP_MOTION_PREDICTION
#define OP_MOTION_PREDICTION

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/BehaviorPrediction.h"
#include "op_utility/DataRW.h"
#include "op_ros_helpers/ROSMapHandler.h"
#include "op_ros_helpers/ROSVelocityHandler.h"
#include "op_ros_helpers/op_ParamsHandler.h"

namespace MotionPredictorNS
{

class MotionPrediction
{
protected:
	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	PlannerHNS::VehicleState m_VehicleStatus;
	bool m_bGoNextStep;

	geometry_msgs::Pose m_OriginPos;
	PlannerHNS::CAR_BASIC_INFO m_CarInfo;
	PlannerHNS::ControllerParams m_ControlParams;
	PlannerHNS::PlanningParams m_PlanningParams;

	std::vector<PlannerHNS::DetectedObject> m_TrackedObjects;
	bool bTrackedObjects;

	PlannerHNS::RoadNetwork m_Map;

	bool m_bEnableCurbObstacles;
	std::vector<PlannerHNS::DetectedObject> curr_curbs_obstacles;

	PlannerHNS::BehaviorPrediction m_PredictBeh;
	autoware_msgs::DetectedObjectArray m_PredictedResultsResults;

	timespec m_VisualizationTimer;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_all_pred_paths;
	std::vector<PlannerHNS::WayPoint> m_particles_points;
	std::vector<PlannerHNS::WayPoint> m_generated_particles_points;

	visualization_msgs::MarkerArray m_PredictedTrajectoriesDummy;
	visualization_msgs::MarkerArray m_PredictedTrajectoriesActual;

	visualization_msgs::MarkerArray m_PredictedParticlesDummy;
	visualization_msgs::MarkerArray m_PredictedParticlesActual;

	visualization_msgs::MarkerArray m_GeneratedParticlesDummy;
	visualization_msgs::MarkerArray m_GeneratedParticlesActual;

	visualization_msgs::MarkerArray m_CurbsDummy;
	visualization_msgs::MarkerArray m_CurbsActual;

	visualization_msgs::MarkerArray m_TargetPointsOnTrajectories;

	double m_DistanceBetweenCurbs;
	double m_VisualizationTime;

	timespec m_SensingTimer;

	std::string m_ExperimentFolderName;
	std::string m_TrackedObjectsTopicName;

	ros::NodeHandle nh;
	ros::Publisher pub_predicted_objects_trajectories;
	ros::Publisher pub_PredictedTrajectoriesRviz ;
	ros::Publisher pub_CurbsRviz ;
	ros::Publisher pub_ParticlesRviz;
	ros::Publisher pub_GeneratedParticlesRviz;
	ros::Publisher pub_BehaviorStateRviz;
	ros::Publisher pub_TargetPointsRviz;

	// define subscribers.
	ros::Subscriber sub_tracked_objects;
	ros::Subscriber sub_current_pose ;
	ros::Subscriber sub_StepSignal;

	// Callback function for subscriber.
	void callbackGetTrackedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetStepForwardSignals(const geometry_msgs::TwistStampedConstPtr& msg);

	//Helper functions
	void VisualizePrediction();
	void UpdatePlanningParams(ros::NodeHandle& _nh);
	void GenerateCurbsObstacles(std::vector<PlannerHNS::DetectedObject>& curb_obstacles);

public:
	MotionPrediction();
	virtual ~MotionPrediction();
	void MainLoop();

	PlannerHNS::MapHandler m_MapHandler;
	PlannerHNS::VelocityHandler m_VelHandler;
	PlannerHNS::ParamsHandler m_ParamsHandler;
};

}

#endif  // OP_MOTION_PREDICTION
