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

#ifndef OP_DIRECT_CONTROLLER_CORE_H
#define OP_DIRECT_CONTROLLER_CORE_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/VehicleCmd.h>

#include "op_planner/control/op_controller.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/PlanningHelpers.h"

namespace op_direct_controller_ns
{

enum STATUS_TYPE{VEHICLE_STATUS, ROBOT_STATUS, SIMULATION_STATUS};

class ControlCommandParams
{
public:
	STATUS_TYPE statusSource;
	bool bTorqueMode; // true -> torque and stroke mode, false -> angle and velocity mode
	bool bAngleMode;
	bool bVelocityMode;
	bool bEnableLogs;
	bool bCalibration;

	ControlCommandParams()
	{
		statusSource = SIMULATION_STATUS;
		bTorqueMode = false;
		bAngleMode = true;
		bVelocityMode = true;
		bEnableLogs = false;
		bCalibration = false;
	}
};

class DirectController
{
protected:

	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;
	PlannerHNS::VehicleState m_VehicleStatus;
	bool bNewVehicleStatus;
	std::vector<PlannerHNS::WayPoint> m_FollowingTrajectory;
	bool bNewTrajectory;
	PlannerHNS::BehaviorState m_CurrentBehavior;
	bool bNewBehaviorState;

	std::vector<double> dt_list;

	PlannerHNS::CAR_BASIC_INFO m_CarInfo;
	PlannerHNS::ControllerParams m_ControlParams;
	PlannerHNS::ControllerHyperParams m_ControllerHyperParams;
	PlannerHNS::MotionControl m_Controller;
	PlannerHNS::ExtendedVehicleState m_TargetStatus;

	ros::NodeHandle nh_;

	ros::Publisher pub_VehicleCommand;
	ros::Publisher pub_ControlInfoRviz;

	// define subscribers.
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_vehicle_status;
	ros::Subscriber sub_behavior_state;
	ros::Subscriber sub_current_trajectory;

	// Callback function for subscriber.
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg);

	//topics from OpenPlanner , op_behavior_selector
	void callbackGetBehaviorState(const autoware_msgs::WaypointConstPtr& msg );
	void callbackGetCurrentTrajectory(const autoware_msgs::LaneConstPtr& msg);

	//Helper Functions
	void UpdateControlParams(ros::NodeHandle& _nh);

public:
	DirectController();
	virtual ~DirectController();
	void MainLoop();
	void displayFollowingInfo(const PlannerHNS::WayPoint& perp_pose, const PlannerHNS::WayPoint& follow_pose, visualization_msgs::MarkerArray& points_markers);
};

}

#endif  // OP_DIRECT_CONTROLLER_CORE_H
