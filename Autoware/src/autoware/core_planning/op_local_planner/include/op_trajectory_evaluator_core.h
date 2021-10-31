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

#ifndef OP_TRAJECTORY_EVALUATOR_CORE
#define OP_TRAJECTORY_EVALUATOR_CORE

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include "op_planner/PlannerCommonDef.h"
#include "op_planner/TrajectoryEvaluator.h"
#include "op_ros_helpers/ROSVelocityHandler.h"
#include "op_ros_helpers/op_ParamsHandler.h"

namespace TrajectoryEvaluatorNS
{

class TrajectoryEvalCore
{
protected:

    PlannerHNS::TrajectoryEvaluator m_TrajectoryCostsCalculator;
	bool m_bUseMoveingObjectsPrediction;

	geometry_msgs::Pose m_OriginPos;

	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	PlannerHNS::VehicleState m_VehicleStatus;
	bool m_bKeepCurrentIfPossible;

	PlannerHNS::VelocityHandler m_VelHandler;
	PlannerHNS::ParamsHandler m_ParamsHandler;

	std::vector<PlannerHNS::WayPoint> m_temp_path;
	std::vector<int> m_CurrGlobalPathsIds;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathsToUse;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathSections;
	std::vector<int> m_prev_index;
	std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_LanesRollOutsToUse;

	std::vector<PlannerHNS::DetectedObject> m_PredictedObjects;
	bool bPredictedObjects;


	struct timespec m_PlanningTimer;
  	std::vector<std::string>    m_LogData;

  	PlannerHNS::EvaluationParams m_EvaluationParams;
  	PlannerHNS::PlanningParams m_PlanningParams;
  	PlannerHNS::PlanningParams m_ModPlanningParams;
  	PlannerHNS::CAR_BASIC_INFO m_CarInfo;

  	PlannerHNS::BehaviorState m_CurrentBehavior;
  	double m_AdditionalFollowDistance;


  	visualization_msgs::MarkerArray m_CollisionsDummy;
	visualization_msgs::MarkerArray m_CollisionsActual;

	std::string m_ExperimentFolderName;
	std::string m_EstimatedObjectsTopicName;

	//ROS messages (topics)
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_CollisionPointsRviz;
	ros::Publisher pub_LocalWeightedTrajectoriesRviz;
	ros::Publisher pub_LocalWeightedTrajectories;
	ros::Publisher pub_TrajectoryCost;
	ros::Publisher pub_SafetyBorderRviz;

	// define subscribers.
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_GlobalPlannerPaths;
	ros::Subscriber sub_LocalPlannerPaths;
	ros::Subscriber sub_predicted_objects;
	ros::Subscriber sub_CurrGlobalLocalPathsIds;


	// Callback function for subscriber.
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg);
	void callbackGetTrajectoryInforFromBehaviorSelector(const std_msgs::Int32MultiArrayConstPtr& msg);

	//Helper Functions
  void UpdatePlanningParams(ros::NodeHandle& _nh);
  //int GetGlobalPathIndex(const int& iCurrTrajectory);
  void CollectRollOutsByGlobalPath(std::vector< std::vector<PlannerHNS::WayPoint> >& local_rollouts);
  void BalanceFactorsToOne(double& priority, double& transition, double& longi, double& lateral, double& change);
  bool FindBestLane(std::vector<PlannerHNS::TrajectoryCost> tcs, PlannerHNS::TrajectoryCost& best_l);
  bool CompareTrajectoriesWithIds(std::vector<std::vector<PlannerHNS::WayPoint> >& paths, std::vector<int>& local_ids);

public:
  TrajectoryEvalCore();
  ~TrajectoryEvalCore();
  void MainLoop();
};

}

#endif  // OP_TRAJECTORY_EVALUATOR_CORE
