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

/// @brief This node connect between HMI remote socket based application and op_globa_planner.
/// we are using state message so we don't create new custom message. and also it is consistent with the socket based communication

#ifndef OP_HMIBRIDGECORE
#define OP_HMIBRIDGECORE

#include <ros/ros.h>
#include <autoware_msgs/State.h>
#include "SocketServer.h"


namespace HMI_BRIDGE_NS
{

class HMI_Bridge
{
public:
	HMI_Bridge();
	virtual ~HMI_Bridge();
	void MainLoop();

protected:
	PlannerHNS::HMI_MSG m_CurrentGlobalMsg;
	HMISocketServer* m_SocketServer;
	ros::Subscriber sub_global_planner;
	ros::Publisher pub_global_planner;
	ros::NodeHandle nh;

	void callbackGetGlobalPlannerState(const autoware_msgs::StateConstPtr &msg);
};

}

#endif  // OP_HMIBRIDGECORE
