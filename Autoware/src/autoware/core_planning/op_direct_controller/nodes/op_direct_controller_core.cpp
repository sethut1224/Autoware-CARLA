/*
 * Copyright 2015-2020 Autoware Foundation. All rights reserved.
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
#include "../include/op_direct_controller.h"


namespace op_direct_controller_ns
{

DirectController::DirectController()
{
	bNewCurrentPos = false;
	bNewTrajectory = false;
	bNewBehaviorState = false;
	bNewVehicleStatus = false;

	ros::NodeHandle _nh;

	UpdateControlParams(_nh);

	pub_ControlInfoRviz = _nh.advertise<visualization_msgs::MarkerArray>("op_direct_controller_points", 1);
	pub_VehicleCommand = _nh.advertise<autoware_msgs::VehicleCmd>("op_controller_cmd", 1);
	// Control Topics Sections
	//----------------------------
	sub_current_pose = _nh.subscribe("/current_pose", 1,	&DirectController::callbackGetCurrentPose, this);
	int bVelSource = 1;
	_nh.getParam("/op_common_params/velocitySource", bVelSource);
	std::string velocity_topic;
	if(bVelSource == 0)
	{
		_nh.getParam("/op_common_params/vel_odom_topic", velocity_topic);
		sub_robot_odom = _nh.subscribe(velocity_topic, 1, &DirectController::callbackGetRobotOdom, this);
	}
	else if(bVelSource == 1)
	{
		_nh.getParam("/op_common_params/vel_curr_topic", velocity_topic);
		sub_current_velocity = _nh.subscribe(velocity_topic, 1, &DirectController::callbackGetAutowareStatus, this);
	}
	else if(bVelSource == 2)
	{
		_nh.getParam("/op_common_params/vel_can_topic", velocity_topic);
		sub_can_info = _nh.subscribe(velocity_topic, 1, &DirectController::callbackGetCANInfo, this);
	}
	else if(bVelSource == 3)
	{
		_nh.getParam("/op_common_params/vehicle_status_topic", velocity_topic);
		sub_vehicle_status = _nh.subscribe(velocity_topic, 1, &DirectController::callbackGetVehicleStatus, this);
	}

	//----------------------------

  	sub_behavior_state = _nh.subscribe("/op_current_behavior",	1, &DirectController::callbackGetBehaviorState, 	this);
  	sub_current_trajectory = _nh.subscribe("/op_local_selected_trajectory", 1,	&DirectController::callbackGetCurrentTrajectory, this);

  	m_ControllerHyperParams.bEnableCalibration = false;
  	m_ControllerHyperParams.bEnableLogs = false;
  	m_ControllerHyperParams.bUseInternalACC = true;
  	m_ControllerHyperParams.bEnableVelocityMode = false;
  	m_ControllerHyperParams.bEnableSteeringMode = false;
  	m_Controller.Init(m_ControlParams, m_ControllerHyperParams, m_CarInfo);//, false, false, false, false);

	std::cout << "OP direct controller initialized successfully " << std::endl;
}

DirectController::~DirectController()
{
}

void DirectController::UpdateControlParams(ros::NodeHandle& nh)
{
	nh.getParam("/op_common_params/width", m_CarInfo.width);
	nh.getParam("/op_common_params/length", m_CarInfo.length);
	nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
	nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
	nh.getParam("/op_common_params/maxWheelAngle", m_CarInfo.max_wheel_angle);
	nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
	nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);
	nh.getParam("/op_common_params/maxVelocity", m_CarInfo.max_speed_forward);
	nh.getParam("/op_common_params/minVelocity", m_CarInfo.min_speed_forward);
	nh.getParam("/op_common_params/steeringDelay", m_ControlParams.SteeringDelay);
	nh.getParam("/op_common_params/minPursuiteDistance", m_ControlParams.minPursuiteDistance );

	nh.getParam("/op_common_params/experimentName", m_ControllerHyperParams.log_file_path);
	if(m_ControllerHyperParams.log_file_path.size() > 0)
	{
		if(m_ControllerHyperParams.log_file_path.at(m_ControllerHyperParams.log_file_path.size()-1) != '/')
		{
			m_ControllerHyperParams.log_file_path.push_back('/');
		}
	}

	int steering_mode = 0;
	int drive_mode = 0;
	nh.getParam("/op_direct_controller/steer_mode", steering_mode );
	nh.getParam("/op_direct_controller/drive_mode", drive_mode);

	nh.getParam("/op_direct_controller/avg_engine_brake_accel", m_ControlParams.avg_engine_brake_accel );
	nh.getParam("/op_direct_controller/min_follow_safe_distance", m_ControlParams.min_safe_follow_distance);

	nh.getParam("/op_direct_controller/control_frequency", m_ControlParams.ControlFrequency );

	nh.getParam("/op_direct_controller/max_steer_value", m_CarInfo.max_steer_value );

	nh.getParam("/op_direct_controller/max_steer_torque", m_CarInfo.max_steer_torque );
	nh.getParam("/op_direct_controller/min_steer_torque", m_CarInfo.min_steer_torque );

	nh.getParam("/op_direct_controller/max_accel_value", m_CarInfo.max_accel_value );
	nh.getParam("/op_direct_controller/max_brake_value", m_CarInfo.max_brake_value );

	nh.getParam("/op_direct_controller/steerGainKP", m_ControlParams.Steering_Gain.kP );
	nh.getParam("/op_direct_controller/steerGainKI", m_ControlParams.Steering_Gain.kI );
	nh.getParam("/op_direct_controller/steerGainKD", m_ControlParams.Steering_Gain.kD );

	nh.getParam("/op_direct_controller/velocityGainKP", m_ControlParams.Velocity_Gain.kP );
	nh.getParam("/op_direct_controller/velocityGainKI", m_ControlParams.Velocity_Gain.kI );
	nh.getParam("/op_direct_controller/velocityGainKD", m_ControlParams.Velocity_Gain.kD );

	nh.getParam("/op_direct_controller/accelGainKP", m_ControlParams.Accel_Gain.kP );
	nh.getParam("/op_direct_controller/accelGainKI", m_ControlParams.Accel_Gain.kI );
	nh.getParam("/op_direct_controller/accelGainKD", m_ControlParams.Accel_Gain.kD );

	nh.getParam("/op_direct_controller/brakeGainKP", m_ControlParams.Brake_Gain.kP );
	nh.getParam("/op_direct_controller/brakeGainKI", m_ControlParams.Brake_Gain.kI );
	nh.getParam("/op_direct_controller/brakeGainKD", m_ControlParams.Brake_Gain.kD );

	nh.getParam("/op_direct_controller/accelInitDelay", m_ControlParams.accel_init_delay );
	nh.getParam("/op_direct_controller/accelAvgDelay", m_ControlParams.accel_avg_delay );
	nh.getParam("/op_direct_controller/avgAcceleration", m_ControlParams.avg_acceleration );

	nh.getParam("/op_direct_controller/brakeInitDelay", m_ControlParams.brake_init_delay );
	nh.getParam("/op_direct_controller/brakeAvgDelay", m_ControlParams.brake_avg_delay );
	nh.getParam("/op_direct_controller/avgDeceleration", m_ControlParams.avg_deceleration );
}

void DirectController::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void DirectController::callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;

	if(fabs(m_CurrentPos.v) > 0.1)
	{
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/m_CurrentPos.v);
	}

	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bNewVehicleStatus = true;
}

void DirectController::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_CurrentPos.v = m_VehicleStatus.speed;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bNewVehicleStatus = true;
}

void DirectController::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed ;
	if(msg->twist.twist.linear.x != 0)
	{
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	}
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bNewVehicleStatus = true;
}

void DirectController::callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = -msg->angle*DEG2RAD;
	m_CurrentPos.v = m_VehicleStatus.speed;
}

void DirectController::callbackGetBehaviorState(const autoware_msgs::WaypointConstPtr& msg )
{
	m_CurrentBehavior = PlannerHNS::ROSHelpers::ConvertAutowareWaypointToBehaviorState(*msg);
	bNewBehaviorState = true;
}

void DirectController::callbackGetCurrentTrajectory(const autoware_msgs::LaneConstPtr &msg)
{
	m_FollowingTrajectory.clear();
	PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(*msg, m_FollowingTrajectory);
	std::cout << "Receive new Trajectory From Behavior Selector : " << msg->waypoints.size() << std::endl;
	bNewTrajectory = true;
}

void DirectController::displayFollowingInfo(const PlannerHNS::WayPoint& perp_pose, const PlannerHNS::WayPoint& follow_pose, visualization_msgs::MarkerArray& points_markers)
{
  visualization_msgs::Marker m1, m3;
  m1.header.frame_id = "map";
  m1.header.stamp = ros::Time();
  m1.ns = "curr_simu_pose";
  m1.type = visualization_msgs::Marker::ARROW;
  m1.action = visualization_msgs::Marker::ADD;
  m1.pose.position.x = perp_pose.pos.x;
  m1.pose.position.y = perp_pose.pos.y;
  m1.pose.position.z = perp_pose.pos.z;
  m1.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(perp_pose.pos.a));
  std_msgs::ColorRGBA green;
  green.a = 1.0;
  green.b = 0.0;
  green.r = 0.0;
  green.g = 1.0;
  m1.color = green;
  m1.scale.x = 1.8;
  m1.scale.y = 0.5;
  m1.scale.z = 0.5;
  m1.frame_locked = true;
  points_markers.markers.push_back(m1);

  m3.header.frame_id = "map";
  m3.header.stamp = ros::Time();
  m3.ns = "follow_pose";
  m3.type = visualization_msgs::Marker::SPHERE;
  m3.action = visualization_msgs::Marker::ADD;
  m3.pose.position.x = follow_pose.pos.x;
  m3.pose.position.y = follow_pose.pos.y;
  m3.pose.position.z = follow_pose.pos.z;
  m3.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(follow_pose.pos.a));
  std_msgs::ColorRGBA red;
  red.a = 1.0;
  red.b = 0.0;
  red.r = 1.0;
  red.g = 0.0;
  m3.color = red;
  m3.scale.x = 0.7;
  m3.scale.y = 0.7;
  m3.scale.z = 0.7;
  m3.frame_locked = true;
  points_markers.markers.push_back(m3);
}

void DirectController::MainLoop()
{
	int freq = m_ControlParams.ControlFrequency;
	ros::Rate loop_rate(freq);

	struct timespec dt_timer;
	UtilityHNS::UtilityH::GetTickCount(dt_timer);
	double dt = 1.0/(double)freq;
	double avg_dt = dt;

	while (ros::ok())
	{
		ros::spinOnce();

		dt = UtilityHNS::UtilityH::GetTimeDiffNow(dt_timer);
		UtilityHNS::UtilityH::GetTickCount(dt_timer);

		dt_list.push_back(dt);
		if(dt_list.size() > freq)
		{
			double dt_sum = 0;
			for(auto& step_dt: dt_list)
			{
				dt_sum += step_dt;
			}
			avg_dt = dt_sum / dt_list.size();
			dt_list.erase(dt_list.begin()+0);
		}

		m_TargetStatus = m_Controller.DoOneStep(avg_dt, m_CurrentBehavior, m_FollowingTrajectory, m_CurrentPos, m_VehicleStatus, bNewTrajectory);

		autoware_msgs::VehicleCmd cmd;
		cmd.steer_cmd.steer = m_TargetStatus.steer_torque;
		cmd.accel_cmd.accel = m_TargetStatus.accel_stroke;
		cmd.brake_cmd.brake = m_TargetStatus.brake_stroke;

		pub_VehicleCommand.publish(cmd);

		visualization_msgs::MarkerArray points_markers;
		displayFollowingInfo(m_Controller.m_PerpendicularPoint, m_Controller.m_FollowMePoint, points_markers);
		pub_ControlInfoRviz.publish(points_markers);

		loop_rate.sleep();
	}
}

}
