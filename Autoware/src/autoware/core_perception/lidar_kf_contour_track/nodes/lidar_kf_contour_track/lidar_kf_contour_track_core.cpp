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

#include "lidar_kf_contour_track_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlannerH.h"
#include "op_planner/KmlMapLoader.h"
#include "op_planner/Lanelet2MapLoader.h"
#include "op_planner/VectorMapLoader.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

std::string output_frame;
std::string input_topic;
std::string output_topic;

namespace ContourTrackerNS
{

ContourTracker::ContourTracker()
{
	m_dt = 0;
	m_tracking_time = 0;
	m_nOriginalPoints = 0;
	m_FilteringTime = 0 ;
	m_FilteringTime = 0;
	m_FilteringTime = 0;
	m_nContourPoints = 0;
	m_PolyEstimationTime = 0;
	bCommonParams = false;
	bNewCurrentPos = false;
	//pointcloud_frame = "velodyne";
	target_tracking_frame = "map";
	kitti_data_dir_ = "~/KITTI_Data/2011_09_26/2011_09_26_drive_0005_sync/";
	result_file_path_ = kitti_data_dir_ + "benchmark_results.txt";
	frame_count_ = 0;
	ReadNodeParams();
	ReadCommonParams();

	m_ObstacleTracking.m_dt = 0.06;
	m_ObstacleTracking.m_bUseCenterOnly = true;
	m_ObstacleTracking.m_Horizon = m_Params.DetectionRadius;
	m_ObstacleTracking.m_bEnableStepByStep = m_Params.bEnableStepByStep;
	m_ObstacleTracking.InitSimpleTracker();

	if(m_Params.bEnableSimulation)
	{
		sub_cloud_clusters = nh.subscribe("/simu_cloud_clusters", 1, &ContourTracker::callbackGetCloudClusters, this);
	}
	else
	{
		sub_detected_objects = nh.subscribe(input_topic, 1, &ContourTracker::callbackGetDetectedObjects, this);
	}
	pub_AllTrackedObjects = nh.advertise<autoware_msgs::DetectedObjectArray>(output_topic, 1);
	sub_current_pose = nh.subscribe("/current_pose",   1, &ContourTracker::callbackGetCurrentPose, 	this);

	m_VelHandler.InitVelocityHandler(nh, PlannerHNS::CAR_BASIC_INFO(), &m_VehicleStatus, &m_CurrentPos);

	if(m_Params.bEnableInternalVisualization)
	{
		pub_DetectedPolygonsRviz = nh.advertise<visualization_msgs::MarkerArray>("/detection/contour_tracker/detected_polygons", 1);
		//pub_TrackedObstaclesRviz = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("op_planner_tracked_boxes", 1);
	}

	if(m_Params.bEnableTTC)
	{
		pub_TTC_PathRviz = nh.advertise<visualization_msgs::MarkerArray>("ttc_direct_path", 1);
	}

	//Mapping Section , load the map if any filtering option is selected
	if(m_Params.filterType != FILTER_DISABLE)
	{
		m_MapHandler.InitMapHandler(nh, "/op_common_params/mapSource",
				"/op_common_params/mapFileName", "/op_common_params/lanelet2_origin");
	}

	m_nDummyObjPerRep = 150;
	m_nDetectedObjRepresentations = 5;
	m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
	m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
	m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
	m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
	m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
	m_DetectedPolygonsActual = m_DetectedPolygonsDummy;
	PlannerHNS::ROSHelpers::InitMarkers(m_nDummyObjPerRep, m_DetectedPolygonsDummy.at(0), m_DetectedPolygonsDummy.at(1), m_DetectedPolygonsDummy.at(2), m_DetectedPolygonsDummy.at(3), m_DetectedPolygonsDummy.at(4));

	m_MatchingInfoDummy.push_back(visualization_msgs::MarkerArray());
	m_MatchingInfoActual = m_MatchingInfoDummy;
	PlannerHNS::ROSHelpers::InitMatchingMarkers(m_nDummyObjPerRep, m_MatchingInfoDummy.at(0));
}

ContourTracker::~ContourTracker()
{
	if(m_Params.bEnableLogging == true)
	{
		std::ostringstream fileName;
		if(m_ExperimentFolderName.size() == 0)
			fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::TrackingFolderName;
		else
			fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_ExperimentFolderName + UtilityHNS::DataRW::TrackingFolderName;

		UtilityHNS::DataRW::WriteLogData(fileName.str(), "contour_tracker",
					"time,dt,num_Tracked_Objects,num_new_objects,num_matched_objects,num_Cluster_Points,num_Contour_Points,t_filtering,t_poly_calc,t_Tracking,t_total",m_LogData);
	}
}

void ContourTracker::ReadNodeParams()
{
	ros::NodeHandle _nh;
	_nh.getParam("/lidar_kf_contour_track/tracking_frame" 			, target_tracking_frame);
	_nh.getParam("/lidar_kf_contour_track/output_frame"				, output_frame);
	_nh.getParam("/lidar_kf_contour_track/tracker_input_topic"		, input_topic);
	_nh.getParam("/lidar_kf_contour_track/tracker_output_topic"		, output_topic);
	_nh.getParam("/lidar_kf_contour_track/vehicle_width" 			, m_Params.VehicleWidth);
	_nh.getParam("/lidar_kf_contour_track/vehicle_length" 			, m_Params.VehicleLength);
	_nh.getParam("/lidar_kf_contour_track/min_object_size" 			, m_Params.MinObjSize);
	_nh.getParam("/lidar_kf_contour_track/max_object_size" 			, m_Params.MaxObjSize);
	_nh.getParam("/lidar_kf_contour_track/polygon_quarters" 		, m_Params.nQuarters);
	_nh.getParam("/lidar_kf_contour_track/polygon_resolution" 		, m_Params.PolygonRes);
	_nh.getParam("/lidar_kf_contour_track/enableSimulationMode" 	, m_Params.bEnableSimulation);
	_nh.getParam("/lidar_kf_contour_track/enableStepByStepMode" 	, m_Params.bEnableStepByStep);
	_nh.getParam("/lidar_kf_contour_track/useDetectionHulls" 		, m_Params.bUseDetectionHulls);


	_nh.getParam("/lidar_kf_contour_track/max_association_distance" , m_ObstacleTracking.m_MAX_ASSOCIATION_DISTANCE);
	_nh.getParam("/lidar_kf_contour_track/max_association_size_diff" , m_ObstacleTracking.m_MAX_ASSOCIATION_SIZE_DIFF);
	_nh.getParam("/lidar_kf_contour_track/enableLogging" , m_Params.bEnableLogging);
	_nh.getParam("/lidar_kf_contour_track/enableInternalVisualization" , m_Params.bEnableInternalVisualization);


	int tracking_type = 0;
	_nh.getParam("/lidar_kf_contour_track/tracking_type" 			, tracking_type);
	if(tracking_type==0)
	{
		m_Params.trackingType = ASSOCIATE_ONLY;
	}
	else if (tracking_type == 1)
	{
		m_Params.trackingType = SIMPLE_TRACKER;
	}
	else if(tracking_type == 2)
	{
		m_Params.trackingType = CONTOUR_TRACKER;
	}

	_nh.getParam("/lidar_kf_contour_track/max_remeber_time" 			, m_ObstacleTracking.m_MaxKeepTime);
	_nh.getParam("/lidar_kf_contour_track/trust_counter" 				, m_ObstacleTracking.m_nMinTrustAppearances);
	_nh.getParam("/lidar_kf_contour_track/vector_map_filter_distance" 	, m_Params.centerlineFilterDistance);

	int filter_type = 0;
	_nh.getParam("/lidar_kf_contour_track/map_filter_type" 	, filter_type);
	if(filter_type == 1)
	{
		m_Params.filterType = FILTER_BOUNDARY;
	}
	else if(filter_type == 2)
	{
		m_Params.filterType = FILTER_CENTERLINES;
	}
	else
	{
		m_Params.filterType = FILTER_DISABLE;
	}
}

void ContourTracker::ReadCommonParams()
{
	ros::NodeHandle _nh("~");
	if(!_nh.getParam("/op_common_params/horizonDistance" , m_Params.DetectionRadius))
	{
		bCommonParams = false;
		m_Params.DetectionRadius = 150;
	}
	else
	{
		bCommonParams = true;
	}

	m_ObstacleTracking.m_CirclesResolution = m_Params.DetectionRadius*0.05;

	if(!_nh.getParam("/op_common_params/enableLaneChange" , m_Params.bEnableLaneChange))
	{
		m_Params.bEnableLaneChange = false;
	}

	_nh.getParam("/op_common_params/experimentName" , m_ExperimentFolderName);

	try
	{
		if(m_ExperimentFolderName.size() > 0)
		{
			if(m_ExperimentFolderName.at(m_ExperimentFolderName.size()-1) != '/')
				m_ExperimentFolderName.push_back('/');
		}

		UtilityHNS::DataRW::CreateLoggingMainFolder();
		if(m_ExperimentFolderName.size() > 1)
		{
			UtilityHNS::DataRW::CreateExperimentFolder(m_ExperimentFolderName);
		}
	}
	catch(exception& e)
	{
		std::cout << "Can't Create Logging folder with experiment name: " << m_ExperimentFolderName << ", Reason: " << e.what() << std::endl;
	}
}

void ContourTracker::dumpResultText(autoware_msgs::DetectedObjectArray& detected_objects)
{
	std::cout << ">>> Benchmark Path : " << result_file_path_ << std::endl;
  std::ofstream outputfile(result_file_path_, std::ofstream::out | std::ofstream::app);
  for(size_t i = 0; i < detected_objects.objects.size(); i++)
  {
    double yaw = tf::getYaw(detected_objects.objects[i].pose.orientation);

    // KITTI tracking benchmark data format:
    // (frame_number,tracked_id, object type, truncation, occlusion, observation angle, x1,y1,x2,y2, h, w, l, cx, cy, cz, yaw)
    // x1, y1, x2, y2 are for 2D bounding box.
    // h, w, l, are for height, width, length respectively
    // cx, cy, cz are for object centroid

    // Tracking benchmark is based on frame_number, tracked_id,
    // bounding box dimentions and object pose(centroid and orientation) from bird-eye view
    outputfile << std::to_string(frame_count_)                               <<" "
               << std::to_string(detected_objects.objects[i].id)             <<" "
               << "Unknown"                                                  <<" "
               << "-1"                                                       <<" "
               << "-1"                                                       <<" "
               << "-1"                                                      <<" "
               << "-1 -1 -1 -1"                                              <<" "
               << std::to_string(detected_objects.objects[i].dimensions.x)   <<" "
               << std::to_string(detected_objects.objects[i].dimensions.y)   <<" "
               << "-1"                                                       <<" "
               << std::to_string(detected_objects.objects[i].pose.position.x)<<" "
               << std::to_string(detected_objects.objects[i].pose.position.y)<<" "
               << "-1"                                                       <<" "
               << std::to_string(yaw)                                        <<"\n";
  }
  frame_count_ ++;
}

void ContourTracker::transformPoseToGlobal(const std::string& src_frame, const std::string& dst_frame, const tf::StampedTransform& local2global, const autoware_msgs::CloudClusterArray& input, autoware_msgs::CloudClusterArray& transformed_input)
{
  for (size_t i = 0; i < input.clusters.size(); i++)
  {
    geometry_msgs::PoseStamped pose_in, pose_out;
    pose_in.pose.position.x = input.clusters.at(i).centroid_point.point.x;
    pose_in.pose.position.y = input.clusters.at(i).centroid_point.point.y;
    pose_in.pose.position.z = input.clusters.at(i).centroid_point.point.z;
    pose_in.pose.orientation = tf::createQuaternionMsgFromYaw(input.clusters.at(i).estimated_angle);

    tf::Transform input_object_pose;
    input_object_pose.setOrigin(tf::Vector3(input.clusters.at(i).centroid_point.point.x, input.clusters.at(i).centroid_point.point.y, input.clusters.at(i).centroid_point.point.z));
    geometry_msgs::Quaternion _qt = tf::createQuaternionMsgFromYaw(input.clusters.at(i).estimated_angle);
    input_object_pose.setRotation(tf::Quaternion(_qt.x, _qt.y, _qt.z, _qt.w));
    tf::poseTFToMsg(local2global * input_object_pose, pose_out.pose);

    autoware_msgs::CloudCluster dd;
    dd = input.clusters.at(i);

    dd.centroid_point.point.x = pose_out.pose.position.x;
    dd.centroid_point.point.y = pose_out.pose.position.y;
    dd.centroid_point.point.z = pose_out.pose.position.z;
    dd.estimated_angle = tf::getYaw(pose_out.pose.orientation);

    transformed_input.clusters.push_back(dd);
  }
}

void ContourTracker::callbackGetDetectedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	if(bNewCurrentPos || m_Params.bEnableSimulation)
	{
		autoware_msgs::DetectedObjectArray globalObjects;
		source_data_frame = msg->header.frame_id;
		m_InputHeader = msg->header;
		//std::cout << "Before Transformation : " << msg->objects.size() << ", " << m_OriginalClusters.size() << std::endl;
		//std::cout << "Source Frame: " << source_data_frame << ", Target Frame: " << target_tracking_frame << std::endl;

		if (source_data_frame.substr(0,1) == "/")
		{
			source_data_frame.erase(source_data_frame.begin());
		}

		if(source_data_frame.compare(target_tracking_frame) > 0)
		{
			PlannerHNS::ROSHelpers::getTransformFromTF(source_data_frame, target_tracking_frame, tf_listener, m_local2global);
			globalObjects.header = msg->header;
			PlannerHNS::ROSHelpers::transformDetectedObjects(source_data_frame, target_tracking_frame, m_local2global, *msg, globalObjects, true);
		}
		else
		{
			globalObjects = *msg;
		}

		//std::cout << "Before Filtering: " << msg->objects.size() << ", " << globalObjects.objects.size() << std::endl;
		ImportDetectedObjects(globalObjects, m_OriginalClusters);
		//std::cout << "Filter the detected Obstacles: " << msg->objects.size() << ", " << m_OriginalClusters.size() << std::endl;

		struct timespec  tracking_timer;
		UtilityHNS::UtilityH::GetTickCount(tracking_timer);

		m_ObstacleTracking.DoOneStep(m_CurrentPos, m_OriginalClusters, m_Params.trackingType);

		m_tracking_time = UtilityHNS::UtilityH::GetTimeDiffNow(tracking_timer);
		m_dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_loop_timer);
		UtilityHNS::UtilityH::GetTickCount(m_loop_timer);

		PostProcess();
	}
}

void ContourTracker::PostProcess()
{
	m_OutPutResults.objects.clear();
	for(unsigned int i = 0 ; i <m_ObstacleTracking.m_DetectedObjects.size(); i++)
	{
		autoware_msgs::DetectedObject obj;
		PlannerHNS::ROSHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(m_ObstacleTracking.m_DetectedObjects.at(i), m_Params.bEnableSimulation, obj);
		m_OutPutResults.objects.push_back(obj);
	}

	// tf::Transform g2ltrans = m_local2global.inverse();
	// tf::StampedTransform global2local;
	// global2local.setData(g2ltrans);

	
	// if(!m_Params.bEnableSimulation)
	// {
	// 	PlannerHNS::ROSHelpers::transformDetectedObjects(target_tracking_frame, source_data_frame , global2local, m_OutPutResults, outPutResultsLocal, true);
	// }
	// else
	// {
	// 	outPutResultsLocal = m_OutPutResults;
	// }

	autoware_msgs::DetectedObjectArray outPutResultsLocal;
	if(target_tracking_frame != output_frame)
	{
		tf::Transform g2ltrans = m_local2global.inverse();
		tf::StampedTransform global2local;
		global2local.setData(g2ltrans);

		PlannerHNS::ROSHelpers::transformDetectedObjects(target_tracking_frame, source_data_frame , global2local, m_OutPutResults, outPutResultsLocal, true);
	}

	else
	{
		outPutResultsLocal = m_OutPutResults;
	}

	outPutResultsLocal.header = m_InputHeader;
	outPutResultsLocal.header.frame_id = output_frame;

	for (int i = 0; i < outPutResultsLocal.objects.size(); i++)
	{
		outPutResultsLocal.objects[i].header = m_InputHeader;
		outPutResultsLocal.objects[i].header.frame_id = output_frame;
	}

	if(m_Params.bEnableInternalVisualization)
	{
		VisualizeLocalTracking();
	}

	pub_AllTrackedObjects.publish(outPutResultsLocal);

}

void ContourTracker::callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr &msg)
{
	if(bNewCurrentPos || m_Params.bEnableSimulation)
	{
		autoware_msgs::CloudClusterArray globalObjects;
		source_data_frame = msg->header.frame_id;
		m_InputHeader = msg->header;

		//std::cout << source_data_frame  << ", " << target_tracking_frame << std::endl;
		if(source_data_frame.compare(target_tracking_frame) > 0)
		{
			PlannerHNS::ROSHelpers::getTransformFromTF(source_data_frame, target_tracking_frame, tf_listener, m_local2global);
			globalObjects.header = msg->header;
			transformPoseToGlobal(source_data_frame, target_tracking_frame, m_local2global, *msg, globalObjects);
		}
		else
		{
			globalObjects = *msg;
		}

		ImportCloudClusters(globalObjects, m_OriginalClusters);

		struct timespec  tracking_timer;
		UtilityHNS::UtilityH::GetTickCount(tracking_timer);

		//std::cout << "Filter the detected Clusters: " << msg->clusters.size() << ", " << m_OriginalClusters.size() << std::endl;

		m_ObstacleTracking.DoOneStep(m_CurrentPos, m_OriginalClusters, m_Params.trackingType);

		m_tracking_time = UtilityHNS::UtilityH::GetTimeDiffNow(tracking_timer);
		m_dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_loop_timer);
		UtilityHNS::UtilityH::GetTickCount(m_loop_timer);

		PostProcess();
	}
}

void ContourTracker::ImportCloudClusters(const autoware_msgs::CloudClusterArray& msg, std::vector<PlannerHNS::DetectedObject>& originalClusters)
{
	originalClusters.clear();
	m_nOriginalPoints = 0;
	m_nContourPoints = 0;
	m_FilteringTime = 0;
	m_PolyEstimationTime = 0;
	struct timespec filter_time, poly_est_time;

	PlannerHNS::DetectedObject obj;
	PlannerHNS::GPSPoint avg_center;
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	if(m_MapHandler.IsMapLoaded())
	{
		m_ClosestLanesList = PlannerHNS::MappingHelpers::GetClosestLanesFast(m_CurrentPos, m_Map, m_Params.DetectionRadius);
	}

	//Filter the detected Obstacles:
	//std::cout << "Filter the detected Obstacles: " << std::endl;
	for(unsigned int i=0; i < msg.clusters.size(); i++)
	{
		obj.center.pos.x = msg.clusters.at(i).centroid_point.point.x;
		obj.center.pos.y = msg.clusters.at(i).centroid_point.point.y;
		obj.center.pos.z = msg.clusters.at(i).centroid_point.point.z;
		obj.center.pos.a = msg.clusters.at(i).estimated_angle;
		obj.center.v = msg.clusters.at(i).score;

		obj.distance_to_center = hypot(obj.center.pos.y-m_CurrentPos.pos.y, obj.center.pos.x-m_CurrentPos.pos.x);

		obj.actual_yaw = msg.clusters.at(i).estimated_angle;

		obj.w = msg.clusters.at(i).dimensions.x;
		obj.l = msg.clusters.at(i).dimensions.y;
		obj.h = msg.clusters.at(i).dimensions.z;

		UtilityHNS::UtilityH::GetTickCount(filter_time);

		//if(!FilterBySize(obj, m_CurrentPos)) continue;
		//if(!FilterByMap(obj, m_CurrentPos, m_Map)) continue;

		m_FilteringTime += UtilityHNS::UtilityH::GetTimeDiffNow(filter_time);

		obj.id = msg.clusters.at(i).id;
		obj.originalID = msg.clusters.at(i).id;
		obj.label = msg.clusters.at(i).label;

		if(msg.clusters.at(i).indicator_state == 0)
			obj.indicator_state = PlannerHNS::INDICATOR_LEFT;
		else if(msg.clusters.at(i).indicator_state == 1)
			obj.indicator_state = PlannerHNS::INDICATOR_RIGHT;
		else if(msg.clusters.at(i).indicator_state == 2)
			obj.indicator_state = PlannerHNS::INDICATOR_BOTH;
		else if(msg.clusters.at(i).indicator_state == 3)
			obj.indicator_state = PlannerHNS::INDICATOR_NONE;


		UtilityHNS::UtilityH::GetTickCount(poly_est_time);
		point_cloud.clear();
		pcl::fromROSMsg(msg.clusters.at(i).cloud, point_cloud);
		PolygonGenerator polyGen(m_Params.nQuarters);
		obj.contour = polyGen.EstimateClusterPolygon(point_cloud ,obj.center.pos, avg_center, m_Params.PolygonRes);

		m_PolyEstimationTime += UtilityHNS::UtilityH::GetTimeDiffNow(poly_est_time);
		m_nOriginalPoints += point_cloud.points.size();
		m_nContourPoints += obj.contour.size();
		originalClusters.push_back(obj);

	}
}

void ContourTracker::ImportDetectedObjects(const autoware_msgs::DetectedObjectArray& msg, std::vector<PlannerHNS::DetectedObject>& originalClusters)
{
	originalClusters.clear();
	m_nOriginalPoints = 0;
	m_nContourPoints = 0;
	m_FilteringTime = 0;
	m_PolyEstimationTime = 0;
	struct timespec filter_time, poly_est_time;


	PlannerHNS::GPSPoint avg_center;
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	if(m_MapHandler.IsMapLoaded())
	{
		m_ClosestLanesList = PlannerHNS::MappingHelpers::GetClosestLanesFast(m_CurrentPos, m_Map, m_Params.DetectionRadius);
	}

	//Filter the detected Obstacles:
	//std::cout << "Filter the detected Obstacles: " << std::endl;
	for(unsigned int i=0; i < msg.objects.size(); i++)
	{
		PlannerHNS::DetectedObject obj;
		obj.center.pos.x = msg.objects.at(i).pose.position.x;
		obj.center.pos.y = msg.objects.at(i).pose.position.y;
		obj.center.pos.z = msg.objects.at(i).pose.position.z;
		obj.center.pos.a = tf::getYaw(msg.objects.at(i).pose.orientation);
		obj.center.v = hypot(msg.objects.at(i).velocity.linear.x,msg.objects.at(i).velocity.linear.y);

		obj.distance_to_center = hypot(obj.center.pos.y-m_CurrentPos.pos.y, obj.center.pos.x-m_CurrentPos.pos.x);

		obj.actual_yaw = obj.center.pos.a;

		obj.w = msg.objects.at(i).dimensions.x;
		obj.l = msg.objects.at(i).dimensions.y;
		obj.h = msg.objects.at(i).dimensions.z;

		UtilityHNS::UtilityH::GetTickCount(filter_time);

		if(!FilterBySize(obj, m_CurrentPos)) continue;
		if(!FilterByMap(obj, m_CurrentPos, m_Map)) continue;

		m_FilteringTime += UtilityHNS::UtilityH::GetTimeDiffNow(filter_time);

		obj.id = msg.objects.at(i).id;
		obj.originalID = msg.objects.at(i).id;
		obj.label = msg.objects.at(i).label;

		if(msg.objects.at(i).indicator_state == 0)
			obj.indicator_state = PlannerHNS::INDICATOR_LEFT;
		else if(msg.objects.at(i).indicator_state == 1)
			obj.indicator_state = PlannerHNS::INDICATOR_RIGHT;
		else if(msg.objects.at(i).indicator_state == 2)
			obj.indicator_state = PlannerHNS::INDICATOR_BOTH;
		else if(msg.objects.at(i).indicator_state == 3)
			obj.indicator_state = PlannerHNS::INDICATOR_NONE;

		if(m_Params.bUseDetectionHulls == true)
		{
			obj.contour.clear();
			for(unsigned int ch_i=0; ch_i<msg.objects.at(i).convex_hull.polygon.points.size(); ch_i++)
			{
				obj.contour.push_back(PlannerHNS::GPSPoint(msg.objects.at(i).convex_hull.polygon.points.at(ch_i).x, msg.objects.at(i).convex_hull.polygon.points.at(ch_i).y, msg.objects.at(i).convex_hull.polygon.points.at(ch_i).z,0));
			}
		}
		else
		{
			UtilityHNS::UtilityH::GetTickCount(poly_est_time);
			point_cloud.clear();
			pcl::fromROSMsg(msg.objects.at(i).pointcloud, point_cloud);
			PolygonGenerator polyGen(m_Params.nQuarters);
			obj.contour = polyGen.EstimateClusterPolygon(point_cloud ,obj.center.pos,avg_center, -1);
			m_PolyEstimationTime += UtilityHNS::UtilityH::GetTimeDiffNow(poly_est_time);
		}

		m_nOriginalPoints += point_cloud.points.size();
		m_nContourPoints += obj.contour.size();
		originalClusters.push_back(obj);

	}
}

bool ContourTracker::FilterByMap(const PlannerHNS::DetectedObject& obj, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map)
{
	if(!m_MapHandler.IsMapLoaded() || m_Params.filterType == FILTER_DISABLE) return true;

	if(m_Params.filterType == FILTER_BOUNDARY)
	{
		for(unsigned int ib = 0; ib < map.boundaries.size(); ib++)
		{
			double d_to_center = hypot(map.boundaries.at(ib).center.pos.y - currState.pos.y, map.boundaries.at(ib).center.pos.x - currState.pos.x);
			if(d_to_center < m_Params.DetectionRadius*2.0)
			{
				if(PlannerHNS::PlanningHelpers::PointInsidePolygon(map.boundaries.at(ib).points, obj.center) == 1)
				{
					return true;
				}
			}
		}
	}
	else if(m_Params.filterType == FILTER_CENTERLINES)
	{
		for(unsigned int i =0 ; i < m_ClosestLanesList.size(); i++)
		{
			PlannerHNS::RelativeInfo info;
			PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(m_ClosestLanesList.at(i)->points, obj.center, info);
			if(info.iFront >= 0  &&  info.iFront < m_ClosestLanesList.at(i)->points.size())
			{

				PlannerHNS::WayPoint wp = m_ClosestLanesList.at(i)->points.at(info.iFront);

				double direct_d = hypot(wp.pos.y - obj.center.pos.y, wp.pos.x - obj.center.pos.x);

			//	std::cout << "- Distance To Car: " << obj.distance_to_center << ", PerpD: " << info.perp_distance << ", DirectD: " << direct_d << ", bAfter: " << info.bAfter << ", bBefore: " << info.bBefore << std::endl;

				if((info.bAfter || info.bBefore) && direct_d > m_Params.centerlineFilterDistance*2.0)
				{
					continue;
				}
			}

			if(fabs(info.perp_distance) <= m_Params.centerlineFilterDistance)
			{
				return true;
			}
		}
	}

	return false;
}

bool ContourTracker::FilterBySize(const PlannerHNS::DetectedObject& obj, const PlannerHNS::WayPoint& currState)
{
	double object_size = hypot(obj.w, obj.l);
	//std::cout << "Object Size: " << object_size  << ", distance to center: " << obj.distance_to_center << ", Radius: " << m_Params.DetectionRadius << std::endl;
	//if(obj.distance_to_center <= m_Params.DetectionRadius && object_size >= m_Params.MinObjSize && object_size <= m_Params.MaxObjSize)
	if(object_size >= m_Params.MinObjSize && object_size <= m_Params.MaxObjSize)
	{
		return true;
	}

//	if(m_Params.bEnableSimulation)
//	{
//		PlannerHNS::Mat3 rotationMat(-currState.pos.a);
//		PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);
//
//		PlannerHNS::GPSPoint relative_point = translationMat*obj.center.pos;
//		relative_point = rotationMat*relative_point;
//
//		double distance_x = fabs(relative_point.x - m_Params.VehicleLength/3.0);
//		double distance_y = fabs(relative_point.y);
//
////		if(distance_x  <= m_Params.VehicleLength*0.5 && distance_y <=  m_Params.VehicleWidth*0.5) // don't detect yourself
////			return false;
//	}

	return false;
}

void ContourTracker::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
  bNewCurrentPos = true;
}

void ContourTracker::VisualizeLocalTracking()
{
	PlannerHNS::ROSHelpers::ConvertTrackedObjectsMarkers(m_CurrentPos, m_ObstacleTracking.m_DetectedObjects,
				m_DetectedPolygonsDummy.at(0),
				m_DetectedPolygonsDummy.at(1),
				m_DetectedPolygonsDummy.at(2),
				m_DetectedPolygonsDummy.at(3),
				m_DetectedPolygonsDummy.at(4),
				m_DetectedPolygonsActual.at(0),
				m_DetectedPolygonsActual.at(1),
				m_DetectedPolygonsActual.at(2),
				m_DetectedPolygonsActual.at(3),
				m_DetectedPolygonsActual.at(4));

	m_DetectedPolygonsAllMarkers.markers.clear();
	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(0).markers.begin(), m_DetectedPolygonsActual.at(0).markers.end());
	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(1).markers.begin(), m_DetectedPolygonsActual.at(1).markers.end());
	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(2).markers.begin(), m_DetectedPolygonsActual.at(2).markers.end());
	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(3).markers.begin(), m_DetectedPolygonsActual.at(3).markers.end());
	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(4).markers.begin(), m_DetectedPolygonsActual.at(4).markers.end());

	//PlannerHNS::ROSHelpers::ConvertMatchingMarkers(m_ObstacleTracking.m_MatchList, m_MatchingInfoDummy.at(0), m_MatchingInfoActual.at(0), 0);
	//m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_MatchingInfoActual.at(0).markers.begin(), m_MatchingInfoActual.at(0).markers.end());

	pub_DetectedPolygonsRviz.publish(m_DetectedPolygonsAllMarkers);

//	jsk_recognition_msgs::BoundingBoxArray boxes_array;
//	boxes_array.header.frame_id = "map";
//	boxes_array.header.stamp  = ros::Time();
//
//	for(unsigned int i = 0 ; i < m_ObstacleTracking.m_DetectedObjects.size(); i++)
//	{
//		jsk_recognition_msgs::BoundingBox box;
//		box.header.frame_id = "map";
//		box.header.stamp = ros::Time().now();
//		box.pose.position.x = m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.x;
//		box.pose.position.y = m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.y;
//		box.pose.position.z = m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.z;
//
//		box.value = 0.9;
//
//		box.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.a+M_PI_2);
//		box.dimensions.x = m_ObstacleTracking.m_DetectedObjects.at(i).l;
//		box.dimensions.y = m_ObstacleTracking.m_DetectedObjects.at(i).w;
//		box.dimensions.z = m_ObstacleTracking.m_DetectedObjects.at(i).h;
//		boxes_array.boxes.push_back(box);
//	}
//
//	pub_TrackedObstaclesRviz.publish(boxes_array);
}

void ContourTracker::Log()
{
	timespec log_t;
	UtilityHNS::UtilityH::GetTickCount(log_t);
	std::ostringstream dataLine;
	std::ostringstream dataLineToOut;
	dataLine << UtilityHNS::UtilityH::GetLongTime(log_t) <<"," << m_dt << "," <<
			m_ObstacleTracking.m_DetectedObjects.size() << "," <<
			m_OriginalClusters.size() << "," <<
			m_ObstacleTracking.m_DetectedObjects.size() - m_OriginalClusters.size() << "," <<
			m_nOriginalPoints << "," <<
			m_nContourPoints<< "," <<
			m_FilteringTime<< "," <<
			m_PolyEstimationTime<< "," <<
			m_tracking_time<< "," <<
			m_tracking_time+m_FilteringTime+m_PolyEstimationTime<< ",";
	m_LogData.push_back(dataLine.str());

	//For Debugging
//	cout << "dt: " << m_dt << endl;
//	cout << "num_Tracked_Objects: " << m_ObstacleTracking.m_DetectedObjects.size() << endl;
//	cout << "num_new_objects: " << m_OriginalClusters.size() << endl;
//	cout << "num_matched_objects: " << m_ObstacleTracking.m_DetectedObjects.size() - m_OriginalClusters.size() << endl;
//	cout << "num_Cluster_Points: " << m_nOriginalPoints << endl;
//	cout << "num_Contour_Points: " << m_nContourPoints << endl;
//	cout << "t_filtering : " << m_FilteringTime << endl;
//	cout << "t_poly_calc : " << m_PolyEstimationTime << endl;
//	cout << "t_Tracking : " << m_tracking_time << endl;
//	cout << "t_total : " << m_tracking_time+m_FilteringTime+m_PolyEstimationTime << endl;
//	cout << endl;

}

void ContourTracker::GetFrontTrajectories(std::vector<PlannerHNS::Lane*>& lanes, const PlannerHNS::WayPoint& currState, const double& max_distance, std::vector<std::vector<PlannerHNS::WayPoint> >& trajectories)
{
	double min_d = DBL_MAX;
	PlannerHNS::WayPoint* pClosest = nullptr;
	for(unsigned int i =0 ; i < lanes.size(); i++)
	{
		PlannerHNS::RelativeInfo info;
		PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(lanes.at(i)->points, currState, info);
		PlannerHNS::WayPoint wp = lanes.at(i)->points.at(info.iFront);

		if(!info.bAfter && !info.bBefore && fabs(info.perp_distance) < min_d)
		{
			min_d = fabs(info.perp_distance);
			pClosest = &lanes.at(i)->points.at(info.iBack);
		}
	}

	if(pClosest == nullptr) return;

	PlannerHNS::PlannerH planner;
	std::vector<PlannerHNS::WayPoint*> closest_pts;
	closest_pts.push_back(pClosest);
	planner.PredictTrajectoriesUsingDP(currState, closest_pts, max_distance, trajectories, false, false);

}

void ContourTracker::CalculateTTC(const std::vector<PlannerHNS::DetectedObject>& objs, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map)
{
	std::vector<std::vector<PlannerHNS::WayPoint> > paths;
	GetFrontTrajectories(m_ClosestLanesList, currState, m_Params.DetectionRadius, paths);

	double min_d = DBL_MAX;
	int closest_obj_id = -1;
	int closest_path_id = -1;
	int i_start = -1;
	int i_end = -1;


	for(unsigned int i_obj = 0; i_obj < objs.size(); i_obj++)
	{
		for(unsigned int i =0 ; i < paths.size(); i++)
		{
			PlannerHNS::RelativeInfo obj_info, car_info;
			PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(paths.at(i), objs.at(i_obj).center , obj_info);

			if(!obj_info.bAfter && !obj_info.bBefore && fabs(obj_info.perp_distance) < m_Params.centerlineFilterDistance)
			{
				PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(paths.at(i), currState , car_info);
				double longitudinalDist = PlannerHNS::PlanningHelpers::GetExactDistanceOnTrajectory(paths.at(i), car_info, obj_info);
				if(longitudinalDist  < min_d)
				{
					min_d = longitudinalDist;
					closest_obj_id = i_obj;
					closest_path_id = i;
					i_start = car_info.iFront;
					i_end = obj_info.iBack;
				}
			}
		}
	}

	std::vector<PlannerHNS::WayPoint> direct_paths;
	if(closest_path_id >= 0 && closest_obj_id >= 0)
	{
		for(unsigned int i=i_start; i<=i_end; i++)
		{
			direct_paths.push_back(paths.at(closest_path_id).at(i));
		}
	}

	//Visualize Direct Path
	m_TTC_Path.markers.clear();
	if(direct_paths.size() == 0)
		direct_paths.push_back(currState);
	PlannerHNS::ROSHelpers::TTC_PathRviz(direct_paths, m_TTC_Path);
	pub_TTC_PathRviz.publish(m_TTC_Path);


	//calculate TTC
	if(direct_paths.size() > 2 && closest_obj_id >= 0)
	{
		double dd = min_d;
		double dv = objs.at(closest_obj_id).center.v - currState.v;
		if(fabs(dv) > 0.1)
		{
			double ttc = (dd - 4) / dv;
			cout << "TTC: " << ttc << ", dv: << " << dv <<", dd:" << dd << endl;
		}
		else
			cout << "TTC: Inf" << endl;
	}
}

void ContourTracker::MainLoop()
{
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		if(!bCommonParams)
		{
			ReadCommonParams();
		}

		//Load the map if any filtering option is selected
		if(m_Params.filterType != FILTER_DISABLE && !m_MapHandler.IsMapLoaded())
		{
			m_MapHandler.LoadMap(m_Map, m_Params.bEnableLaneChange);
		}

		/**
		 * Main loop happens when new detected object topic is received,
		 * so the frequency of this node depends on the object detection module frequency.
		 */
		ros::spinOnce();
		loop_rate.sleep();
	}
}

}
