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

#include "op_motion_predictor_core.h"
#include "op_planner/MappingHelpers.h"
#include "op_ros_helpers/op_ROSHelpers.h"

namespace MotionPredictorNS
{

MotionPrediction::MotionPrediction()
{
	bNewCurrentPos = false;
	bTrackedObjects = false;
	m_bEnableCurbObstacles = false;
	m_DistanceBetweenCurbs = 1.0;
	m_VisualizationTime = 0.25;
	m_bGoNextStep = false;

	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	tf::TransformListener tf_listener;
	PlannerHNS::ROSHelpers::getTransformFromTF("world", "map", tf_listener, transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_predicted_objects_trajectories = nh.advertise<autoware_msgs::DetectedObjectArray>("/predicted_objects", 1);
	pub_PredictedTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("/predicted_trajectories_rviz", 1);
	pub_CurbsRviz					= nh.advertise<visualization_msgs::MarkerArray>("/map_curbs_rviz", 1);
	pub_ParticlesRviz = nh.advertise<visualization_msgs::MarkerArray>("prediction_particles", 1);
	pub_GeneratedParticlesRviz = nh.advertise<visualization_msgs::MarkerArray>("generated_particles", 1);
	pub_BehaviorStateRviz = nh.advertise<visualization_msgs::MarkerArray>("prediction_behaviors", 1);
	pub_TargetPointsRviz = nh.advertise<visualization_msgs::MarkerArray>("target_points_on_trajs", 1);

//	sub_StepSignal = nh.subscribe("/pred_step_signal", 		1, &MotionPrediction::callbackGetStepForwardSignals, 		this);
	sub_tracked_objects	= nh.subscribe(m_TrackedObjectsTopicName, 	1,	&MotionPrediction::callbackGetTrackedObjects, 		this);
	sub_current_pose 	= nh.subscribe("/current_pose", 1,	&MotionPrediction::callbackGetCurrentPose, 		this);

	m_VelHandler.InitVelocityHandler(nh, m_CarInfo, &m_VehicleStatus, &m_CurrentPos);
	m_ParamsHandler.InitHandler(_nh, &m_PlanningParams, &m_CarInfo, &m_ControlParams);

	UtilityHNS::UtilityH::GetTickCount(m_VisualizationTimer);
	PlannerHNS::ROSHelpers::InitPredMarkers(500, m_PredictedTrajectoriesDummy);
	PlannerHNS::ROSHelpers::InitCurbsMarkers(500, m_CurbsDummy);
	PlannerHNS::ROSHelpers::InitPredParticlesMarkers(1000, m_PredictedParticlesDummy);
	PlannerHNS::ROSHelpers::InitPredParticlesMarkers(2000, m_GeneratedParticlesDummy, true);

	m_MapHandler.InitMapHandler(nh, "/op_common_params/mapSource",
			"/op_common_params/mapFileName", "/op_common_params/lanelet2_origin");

	std::cout << "OpenPlanner Motion Predictor initialized successfully " << std::endl;
}

MotionPrediction::~MotionPrediction()
{
}

void MotionPrediction::UpdatePlanningParams(ros::NodeHandle& _nh)
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

	std::cout << "Rolls Number: " << m_PlanningParams.rollOutNumber << std::endl;

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

	_nh.getParam("/op_common_params/objects_input_topic" , m_TrackedObjectsTopicName);
	if(m_TrackedObjectsTopicName.empty())
	{
		m_TrackedObjectsTopicName = "/detection/contour_tracker/objects";
	}

	_nh.getParam("/op_common_params/experimentName" , m_ExperimentFolderName);
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

	_nh.getParam("/op_motion_predictor/enableGenrateBranches" , m_PredictBeh.m_bGenerateBranches);
	_nh.getParam("/op_motion_predictor/max_distance_to_lane" , m_PredictBeh.m_LaneDetectionDistance);
	_nh.getParam("/op_motion_predictor/min_prediction_distance" , m_PredictBeh.m_MinPredictionDistance);
	_nh.getParam("/op_motion_predictor/min_prediction_time" , m_PredictBeh.m_MinPredictionTime);
	_nh.getParam("/op_motion_predictor/enableCurbObstacles"	, m_bEnableCurbObstacles);
	_nh.getParam("/op_motion_predictor/distanceBetweenCurbs", m_DistanceBetweenCurbs);
	_nh.getParam("/op_motion_predictor/visualizationTime", m_VisualizationTime);
	_nh.getParam("/op_motion_predictor/enableStepByStepSignal", 	m_PredictBeh.m_bStepByStep );
	if(m_PredictBeh.m_bStepByStep)
		m_PredictBeh.m_bDebugOut = true;

	_nh.getParam("/op_motion_predictor/enableParticleFilterPrediction", 	m_PredictBeh.m_bParticleFilter);

	m_PredictBeh.g_PredParams.experiment_name = m_ExperimentFolderName;
	std::cout << "Particles Num Before : " <<  m_PredictBeh.g_PredParams.MAX_PARTICLES_NUM << std::endl;
	_nh.getParam("/op_motion_predictor/pose_weight_factor", 	m_PredictBeh.g_PredParams.POSE_FACTOR);
	_nh.getParam("/op_motion_predictor/dir_weight_factor", 	m_PredictBeh.g_PredParams.DIRECTION_FACTOR);
	_nh.getParam("/op_motion_predictor/vel_weight_factor", 	m_PredictBeh.g_PredParams.VELOCITY_FACTOR);
	_nh.getParam("/op_motion_predictor/acc_weight_factor", 	m_PredictBeh.g_PredParams.ACCELERATE_FACTOR);
	_nh.getParam("/op_motion_predictor/ind_weight_factor", 	m_PredictBeh.g_PredParams.INDICATOR_FACTOR);

	_nh.getParam("/op_motion_predictor/particles_number", 	m_PredictBeh.g_PredParams.MAX_PARTICLES_NUM);
	_nh.getParam("/op_motion_predictor/min_particles_num", 	m_PredictBeh.g_PredParams.MIN_PARTICLES_NUM);
	_nh.getParam("/op_motion_predictor/keep_percentage", 	m_PredictBeh.g_PredParams.KEEP_PERCENTAGE);
	m_PredictBeh.SetForTrajTracker();

	UtilityHNS::UtilityH::GetTickCount(m_SensingTimer);
}

void MotionPrediction::callbackGetStepForwardSignals(const geometry_msgs::TwistStampedConstPtr& msg)
{
	if(msg->twist.linear.x == 1)
		m_bGoNextStep = true;
	else
		m_bGoNextStep = false;
}

void MotionPrediction::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void MotionPrediction::callbackGetTrackedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	UtilityHNS::UtilityH::GetTickCount(m_SensingTimer);

	autoware_msgs::DetectedObjectArray globalObjects;
	std::string source_data_frame = msg->header.frame_id;
	std::string target_prediction_frame = "/map";

	if (source_data_frame.substr(0, 1) == "/")
	{
		source_data_frame.erase(source_data_frame.begin());
	}

	if(source_data_frame.compare(target_prediction_frame) > 0)
	{
		tf::TransformListener tf_listener;
		tf::StampedTransform local2global;
		PlannerHNS::ROSHelpers::getTransformFromTF(source_data_frame, target_prediction_frame, tf_listener, local2global);
		globalObjects.header = msg->header;
		PlannerHNS::ROSHelpers::transformDetectedObjects(source_data_frame, target_prediction_frame, local2global, *msg, globalObjects);
	}
	else
	{
		globalObjects = *msg;
	}


//	std::cout << std::endl << "New : " << globalObjects.objects.size() << ", Old: " << m_TrackedObjects.size() << std::endl << std::endl;

	m_TrackedObjects.clear();
	bTrackedObjects = true;

	PlannerHNS::DetectedObject obj;
	for(unsigned int i = 0 ; i <globalObjects.objects.size(); i++)
	{
		if(globalObjects.objects.at(i).id > 0)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(globalObjects.objects.at(i), obj);
			m_TrackedObjects.push_back(obj);
		}
	}

	if(m_PredictBeh.m_bStepByStep && m_bGoNextStep)
	{
		m_bGoNextStep = false;
		m_PredictBeh.DoOneStep(m_TrackedObjects, m_CurrentPos, m_PlanningParams.minSpeed, m_CarInfo.max_deceleration,  m_Map);
	}
	else if(!m_PredictBeh.m_bStepByStep)
	{
		m_PredictBeh.DoOneStep(m_TrackedObjects, m_CurrentPos, m_PlanningParams.minSpeed, m_CarInfo.max_deceleration,  m_Map);
	}

	m_PredictedResultsResults.objects.clear();
	autoware_msgs::DetectedObject pred_obj;
	for(unsigned int i = 0 ; i <m_PredictBeh.m_ParticleInfo.size(); i++)
	{
		PlannerHNS::ROSHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(m_PredictBeh.m_ParticleInfo.at(i)->obj, false, pred_obj);
		if(m_PredictBeh.m_ParticleInfo.at(i)->best_behavior_track)
			pred_obj.behavior_state = m_PredictBeh.m_ParticleInfo.at(i)->best_behavior_track->best_beh_by_p;
		m_PredictedResultsResults.objects.push_back(pred_obj);
	}

	if(m_bEnableCurbObstacles)
	{
		curr_curbs_obstacles.clear();
		GenerateCurbsObstacles(curr_curbs_obstacles);
		PlannerHNS::ROSHelpers::ConvertCurbsMarkers(curr_curbs_obstacles, m_CurbsActual, m_CurbsDummy);
		pub_CurbsRviz.publish(m_CurbsActual);
		//std::cout << "Curbs No: " << curr_curbs_obstacles.size() << endl;
		for(unsigned int i = 0 ; i <curr_curbs_obstacles.size(); i++)
		{
			PlannerHNS::ROSHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(curr_curbs_obstacles.at(i), false, pred_obj);
			m_PredictedResultsResults.objects.push_back(pred_obj);
		}
	}

	m_PredictedResultsResults.header.stamp = ros::Time().now();
	pub_predicted_objects_trajectories.publish(m_PredictedResultsResults);

	VisualizePrediction();
}

void MotionPrediction::GenerateCurbsObstacles(std::vector<PlannerHNS::DetectedObject>& curb_obstacles)
{
	if(!bNewCurrentPos) return;
	PlannerHNS::DetectedObject obj;

	for(unsigned int ic = 0; ic < m_Map.curbs.size(); ic++)
	{
		bool bCloseCurb = false;
		for(unsigned int icp=0; icp< m_Map.curbs.at(ic).points.size(); icp++)
		{
			PlannerHNS::WayPoint* pP = &m_Map.curbs.at(ic).points.at(icp);
			double distance = hypot(m_CurrentPos.pos.y - pP->pos.y, m_CurrentPos.pos.x - pP->pos.x);

			if(distance < m_PlanningParams.microPlanDistance)
			{
				bCloseCurb = true;
				break;
			}
		}

		if(bCloseCurb)
		{
			obj.contour.clear();
			for(auto& p: m_Map.curbs.at(ic).points)
			{
				obj.contour.push_back(p.pos);
			}
			PlannerHNS::PlanningHelpers::FixPathDensity(obj.contour, m_DistanceBetweenCurbs);
			obj.bDirection = false;
			obj.bVelocity = false;
			obj.id = -1;
			obj.t  = PlannerHNS::SIDEWALK;
			obj.label = "curb";
			curb_obstacles.push_back(obj);
		}
	}
}

void MotionPrediction::VisualizePrediction()
{
//	m_all_pred_paths.clear();
//	for(unsigned int i=0; i< m_PredictBeh.m_PredictedObjects.size(); i++)
//		m_all_pred_paths.insert(m_all_pred_paths.begin(), m_PredictBeh.m_PredictedObjects.at(i).predTrajectories.begin(), m_PredictBeh.m_PredictedObjects.at(i).predTrajectories.end());
//
//	PlannerHNS::ROSHelpers::ConvertPredictedTrqajectoryMarkers(m_all_pred_paths, m_PredictedTrajectoriesActual, m_PredictedTrajectoriesDummy);
//	pub_PredictedTrajectoriesRviz.publish(m_PredictedTrajectoriesActual);
//


	m_all_pred_paths.clear();
	m_particles_points.clear();
	visualization_msgs::MarkerArray behavior_rviz_arr;


	m_TargetPointsOnTrajectories.markers.clear();

	for(unsigned int i=0; i< m_PredictBeh.m_ParticleInfo.size(); i++)
	{
		m_all_pred_paths.insert(m_all_pred_paths.begin(), m_PredictBeh.m_ParticleInfo.at(i)->obj.predTrajectories.begin(), m_PredictBeh.m_ParticleInfo.at(i)->obj.predTrajectories.end());

		for(unsigned int t=0; t < m_PredictBeh.m_ParticleInfo.at(i)->m_TrajectoryTracker.size(); t++)
		{
			PlannerHNS::WayPoint tt_wp = m_PredictBeh.m_ParticleInfo.at(i)->m_TrajectoryTracker.at(t)->followPoint;
			visualization_msgs::Marker targetPoint = PlannerHNS::ROSHelpers::CreateGenMarker(tt_wp.pos.x,tt_wp.pos.y,tt_wp.pos.z,0,0,0.0,1,0.5,t,"target_trajectory_point", visualization_msgs::Marker::SPHERE);
			m_TargetPointsOnTrajectories.markers.push_back(targetPoint);

			PlannerHNS::WayPoint p_wp;
			for(unsigned int j=0; j < m_PredictBeh.m_ParticleInfo.at(i)->m_TrajectoryTracker.at(t)->m_CurrParts.size(); j++)
			{
				PlannerHNS::Particle* pPart = &m_PredictBeh.m_ParticleInfo.at(i)->m_TrajectoryTracker.at(t)->m_CurrParts.at(j);
				p_wp = pPart->pose;
				if(pPart->beh == PlannerHNS::BEH_STOPPING_STATE)
					p_wp.bDir = PlannerHNS::STANDSTILL_DIR;
				else if(pPart->beh == PlannerHNS::BEH_PARKING_STATE)
					p_wp.bDir = PlannerHNS::STANDSTILL_DIR;
				else if(pPart->beh == PlannerHNS::BEH_YIELDING_STATE)
					p_wp.bDir = PlannerHNS::BACKWARD_DIR;
				else if(pPart->beh == PlannerHNS::BEH_FORWARD_STATE)
					p_wp.bDir = PlannerHNS::FORWARD_DIR;
				else if(pPart->beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
					p_wp.bDir = PlannerHNS::FORWARD_LEFT_DIR;
				else if(pPart->beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
					p_wp.bDir = PlannerHNS::FORWARD_RIGHT_DIR;

				m_particles_points.push_back(p_wp);
			}
		}

		if(m_PredictBeh.m_ParticleInfo.at(i) != nullptr && m_PredictBeh.m_ParticleInfo.at(i)->best_behavior_track != nullptr)
		{
			visualization_msgs::Marker behavior_rviz;
			std::ostringstream ns_beh;
			ns_beh << "pred_beh_state_" << i;
			PlannerHNS::ROSHelpers::VisualizeIntentionState(m_PredictBeh.m_ParticleInfo.at(i)->obj.center, m_PredictBeh.m_ParticleInfo.at(i)->best_behavior_track->best_beh_by_p, behavior_rviz, ns_beh.str(), 3);
			behavior_rviz_arr.markers.push_back(behavior_rviz);
		}
	}
	pub_BehaviorStateRviz.publish(behavior_rviz_arr);


	PlannerHNS::ROSHelpers::ConvertParticles(m_particles_points,m_PredictedParticlesActual, m_PredictedParticlesDummy);
	pub_ParticlesRviz.publish(m_PredictedParticlesActual);

	//std::cout << "Start Tracking of Trajectories : " <<  m_all_pred_paths.size() << endl;
	for(auto& path: m_all_pred_paths)
	{
		PlannerHNS::PlanningHelpers::FixPathDensity(path, 1.0);
	}

	PlannerHNS::ROSHelpers::ConvertPredictedTrqajectoryMarkers(m_all_pred_paths, m_PredictedTrajectoriesActual, m_PredictedTrajectoriesDummy);
	pub_PredictedTrajectoriesRviz.publish(m_PredictedTrajectoriesActual);

	m_generated_particles_points.clear();
	for(unsigned int i=0; i< m_PredictBeh.m_ParticleInfo.size(); i++)
	{
		PlannerHNS::WayPoint p_wp;
		for(unsigned int t=0; t < m_PredictBeh.m_ParticleInfo.at(i)->m_AllGeneratedParticles.size(); t++)
		{
			p_wp = m_PredictBeh.m_ParticleInfo.at(i)->m_AllGeneratedParticles.at(t).pose;
			if(m_PredictBeh.m_ParticleInfo.at(i)->m_AllGeneratedParticles.at(t).beh == PlannerHNS::BEH_STOPPING_STATE)
			{
				p_wp.bDir = PlannerHNS::STANDSTILL_DIR;
			}
			else if(m_PredictBeh.m_ParticleInfo.at(i)->m_AllGeneratedParticles.at(t).beh == PlannerHNS::BEH_FORWARD_STATE)
			{
				p_wp.bDir = PlannerHNS::FORWARD_DIR;
			}
			else if(m_PredictBeh.m_ParticleInfo.at(i)->m_AllGeneratedParticles.at(t).beh == PlannerHNS::BEH_YIELDING_STATE)
			{
				p_wp.bDir = PlannerHNS::BACKWARD_DIR;
			}
			m_generated_particles_points.push_back(p_wp);
		}
	}
	PlannerHNS::ROSHelpers::ConvertParticles(m_generated_particles_points,m_GeneratedParticlesActual, m_GeneratedParticlesDummy, true);
	pub_GeneratedParticlesRviz.publish(m_GeneratedParticlesActual);


	pub_TargetPointsRviz.publish(m_TargetPointsOnTrajectories);
}

void MotionPrediction::MainLoop()
{
	ros::Rate loop_rate(50);

	while (ros::ok())
	{
		ros::spinOnce();

		if(!m_MapHandler.IsMapLoaded())
		{
			m_MapHandler.LoadMap(m_Map, m_PlanningParams.enableLaneChange);
		}

//		if(UtilityHNS::UtilityH::GetTimeDiffNow(m_VisualizationTimer) > m_VisualizationTime)
//		{
//			VisualizePrediction();
//			UtilityHNS::UtilityH::GetTickCount(m_VisualizationTimer);
//		}

		//For the debugging of prediction
//		if(UtilityHNS::UtilityH::GetTimeDiffNow(m_SensingTimer) > 5)
//		{
//			ROS_INFO("op_motion_prediction sensing timeout, can't receive tracked object data ! Reset .. Reset");
//			m_PredictedResultsResults.objects.clear();
//			pub_predicted_objects_trajectories.publish(m_PredictedResultsResults);
//		}

		loop_rate.sleep();
	}
}

}
