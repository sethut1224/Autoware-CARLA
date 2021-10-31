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

#ifndef KF_CONTOUR_TRACKER_CORE
#define KF_CONTOUR_TRACKER_CORE

// ROS includes
#include <ros/ros.h>

#include "op_planner/RoadNetwork.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/MatrixOperations.h"
#include "SimpleTracker.h"
#include "PolygonGenerator.h"
#include "op_ros_helpers/ROSMapHandler.h"
#include "op_ros_helpers/ROSVelocityHandler.h"

#include <tf/transform_listener.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

namespace ContourTrackerNS
{

enum MAP_FILTER_TYPE
{
	FILTER_DISABLE,
	FILTER_BOUNDARY,
	FILTER_CENTERLINES
	};

class PerceptionParams
{
public:

	double 	VehicleWidth;
	double 	VehicleLength;
	double 	DetectionRadius;
	double 	MinObjSize;
	double 	MaxObjSize;
	double  nQuarters;
	double 	PolygonRes;
	TRACKING_TYPE	trackingType; // 0 association only , 1 simple tracking, 2 contour based tracking
	bool bEnableSimulation;
	bool bEnableStepByStep;
	bool bEnableLogging;
	bool bEnableTTC;
	bool bEnableLaneChange;
	bool bEnableBenchmark;
	bool bEnableInternalVisualization;
	bool bUseDetectionHulls;
	MAP_FILTER_TYPE filterType;
	double centerlineFilterDistance;

	PerceptionParams()
	{
		VehicleWidth =0;
		VehicleLength =0;
		DetectionRadius =0;
		MinObjSize =0;
		MaxObjSize =0;
		nQuarters = 0;
		PolygonRes = 0;
		trackingType = SIMPLE_TRACKER;
		bEnableStepByStep = false;
		bEnableSimulation = false;
		bEnableLogging = false;
		bEnableTTC = false;
		bEnableLaneChange = false;
		bEnableBenchmark = false;
		bEnableInternalVisualization = false;
		bUseDetectionHulls = false;
		filterType = FILTER_DISABLE;
		centerlineFilterDistance = 1.5;
	}
};

class ContourTracker
{
protected:
	std::vector<PlannerHNS::DetectedObject> m_OriginalClusters;
	autoware_msgs::DetectedObjectArray m_OutPutResults;
	bool bNewClusters;
	PlannerHNS::WayPoint m_CurrentPos;
	PlannerHNS::VehicleState m_VehicleStatus;
	bool bNewCurrentPos;
	PerceptionParams m_Params;
	SimpleTracker m_ObstacleTracking;

	//Visualization Section
	int m_nDummyObjPerRep;
	int m_nDetectedObjRepresentations;
	std::vector<visualization_msgs::MarkerArray> m_DetectedPolygonsDummy;
	std::vector<visualization_msgs::MarkerArray> m_DetectedPolygonsActual;
	visualization_msgs::MarkerArray m_DetectedPolygonsAllMarkers;
	visualization_msgs::MarkerArray m_DetectionCircles;

	std::vector<visualization_msgs::MarkerArray> m_MatchingInfoDummy;
	std::vector<visualization_msgs::MarkerArray> m_MatchingInfoActual;


	visualization_msgs::MarkerArray m_TTC_Path;
	visualization_msgs::Marker m_TTC_Info;

	std::vector<std::string>    m_LogData;
	PlannerHNS::MapHandler m_MapHandler;
	PlannerHNS::VelocityHandler m_VelHandler;
	PlannerHNS::RoadNetwork m_Map;
	bool bCommonParams;
	std::string m_ExperimentFolderName;

	std::vector<PlannerHNS::Lane*> m_ClosestLanesList;

	int m_nOriginalPoints;
	int m_nContourPoints;
	double m_FilteringTime;
	double m_PolyEstimationTime;
	double m_tracking_time;
	double m_dt;
	struct timespec  m_loop_timer;
	int frame_count_;
	std::string kitti_data_dir_;
	std::string result_file_path_;
	//std::string pointcloud_frame ;
	std::string target_tracking_frame;
	std::string source_data_frame;
	tf::TransformListener tf_listener;
	tf::StampedTransform m_local2global;
	std_msgs::Header m_InputHeader;

	//ROS subscribers
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_AllTrackedObjects;

	ros::Publisher pub_DetectedPolygonsRviz;
	//ros::Publisher pub_TrackedObstaclesRviz;
	ros::Publisher pub_TTC_PathRviz;

	// define subscribers.
	ros::Subscriber sub_cloud_clusters;
	ros::Subscriber sub_current_pose ;
	ros::Subscriber sub_detected_objects;

	// Callback function for subscriber.
	void callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr &msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetDetectedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg);

	//Helper Functions
	void VisualizeLocalTracking();
	void ImportCloudClusters(const autoware_msgs::CloudClusterArray& msg, std::vector<PlannerHNS::DetectedObject>& originalClusters);
	void ImportDetectedObjects(const autoware_msgs::DetectedObjectArray& msg, std::vector<PlannerHNS::DetectedObject>& originalClusters);
	bool FilterByMap(const PlannerHNS::DetectedObject& obj, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map);
	bool FilterBySize(const PlannerHNS::DetectedObject& obj, const PlannerHNS::WayPoint& currState);
	void CalculateTTC(const std::vector<PlannerHNS::DetectedObject>& objs, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map);
	void GetFrontTrajectories(std::vector<PlannerHNS::Lane*>& lanes, const PlannerHNS::WayPoint& currState, const double& max_distance, std::vector<std::vector<PlannerHNS::WayPoint> >& trajectories);
	void transformPoseToGlobal(const std::string& src_frame, const std::string& dst_frame, const tf::StampedTransform& local2global, const autoware_msgs::CloudClusterArray& input, autoware_msgs::CloudClusterArray& transformed_input);
	void ReadNodeParams();
	void ReadCommonParams();
	void Log();
	void PostProcess();
	void dumpResultText(autoware_msgs::DetectedObjectArray& detected_objects);

public:
  ContourTracker();
  ~ContourTracker();
  void MainLoop();
};

}

#endif  // KF_CONTOUR_TRACKER_CORE
