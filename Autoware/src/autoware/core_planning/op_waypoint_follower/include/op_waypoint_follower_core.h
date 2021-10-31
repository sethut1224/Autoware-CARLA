/**
 * @author Hatem Darweesh
 * @date 2-Jan-2021
 */

#ifndef OP_WAYPOINT_FOLLOWER_CORE_H
#define OP_WAYPOINT_FOLLOWER_CORE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/VehicleCmd.h"
#include "op_planner/PlannerH.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/control/op_controller.h"
#include "op_planner/control/MotionSimulator.h"
#include "op_ros_helpers/ROSVelocityHandler.h"

namespace op_waypoint_follower
{

#define _AVOID_USING_TWIST_FILTER

class TopicNamesParams
{
public:
	std::string pose_topic_name;
	std::string localizer_pose_topic_name;
	std::string velocity_topic_name;
	std::string vehicle_status_topic_name;
};

class WaypointFollower
{
protected:
	PlannerHNS::WayPoint m_InitPos;
	bool bInitPos;
	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;
	bool bNewTrajectory;
	bool bNewBehaviorState;
	PlannerHNS::BehaviorState m_CurrentBehavior;
	bool bNewVelocity;
	PlannerHNS::MotionSimulator m_State;

	autoware_msgs::VehicleCmd m_CurrVehicleCommand;
	bool bNewVehicleCmd;
	PlannerHNS::VehicleState m_CurrVehicleStatus;

	std::vector<double> dt_list;
	PlannerHNS::VelocityHandler m_VelHandler;
	bool bVehicleStatus;
	PlannerHNS::ControllerHyperParams m_ControllerHyperParams;
	TopicNamesParams m_Topics;
	PlannerHNS::CAR_BASIC_INFO m_CarInfo;
	PlannerHNS::ControllerParams m_ControlParams;
	PlannerHNS::PlanningParams m_PlanningParams;

	PlannerHNS::MotionControl m_PathFollower;
	std::vector<PlannerHNS::WayPoint> m_Path;

	PlannerHNS::ExtendedVehicleState m_curr_target_status;
	PlannerHNS::VehicleState m_curr_motion_status;

	int m_iSimulationMode;

	ros::Publisher pub_TwistRaw;
	ros::Publisher pub_ControlRaw;
	ros::Publisher pub_SimuLocalizerPose;
	ros::Publisher pub_SimuPose;
	ros::Publisher pub_SimuVelocity;
	ros::Publisher pub_SimuVehicleStatus;
	ros::Publisher pub_CurrPoseRviz;
	ros::Publisher pub_ForwardSimuPoseRviz;
	ros::Publisher pub_FollowPointRviz;
	ros::Publisher pub_VelocityRviz;
	ros::Publisher pub_VehicleCommandOP;

	// define subscribers.
	ros::Subscriber sub_initialpose;
	ros::Subscriber sub_current_pose ;
	ros::Subscriber sub_behavior_state;
	ros::Subscriber sub_current_trajectory;
	ros::Subscriber sub_vehicle_command;

	// Callback function for subscriber.
	void callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetBehaviorState(const autoware_msgs::WaypointConstPtr& msg);
	void callbackGetCurrentTrajectory(const autoware_msgs::LaneConstPtr& msg);

	void callbackVehicleCommand(const autoware_msgs::VehicleCmdConstPtr& msg);

	void SimulatedMoveStep(double dt);
	void PathFollowingStep(double dt);

public:
	WaypointFollower();
	virtual ~WaypointFollower();
	void RunMainLoop();

	void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);
	void ReadParamFromLaunchFile(ros::NodeHandle& nh);
	void displayFollowingInfo();
};

}

#endif  // OP_WAYPOINT_FOLLOWER_CORE_H
