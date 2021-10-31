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

#include "op_hmi_bridge_core.h"
#include "op_utility/DataRW.h"
namespace HMI_BRIDGE_NS
{

HMI_Bridge::HMI_Bridge()
{
	int send_port = 0;
	int receive_port = 0;

	nh.getParam("/op_hmi_bridge/send_port" , send_port);
	nh.getParam("/op_hmi_bridge/receive_port" , receive_port);

	pub_global_planner = nh.advertise<autoware_msgs::State>("/hmi_mission_command", 1, true);
	sub_global_planner = nh.subscribe("/op_mission_status", 1, &HMI_Bridge::callbackGetGlobalPlannerState, 		this);

	m_SocketServer = new HMISocketServer;
	while(m_SocketServer->InitSocket(send_port, receive_port) < 0)
	{
		usleep(1000);
	}

	std::cout << "HMI Bridge initialized successfully " << std::endl;
}

HMI_Bridge::~HMI_Bridge()
{
	if(m_SocketServer != nullptr)
	{
		delete m_SocketServer;
		m_SocketServer = nullptr;
	}
}

void HMI_Bridge::callbackGetGlobalPlannerState(const autoware_msgs::StateConstPtr &msg)
{
	m_CurrentGlobalMsg = PlannerHNS::HMI_MSG::FromString(msg->mission_state);

	// Send the latest global planning message to socket based HMI
	if(m_SocketServer != nullptr)
	{
		//std::cout << "Received Message From GP:  " << msg->mission_state <<std::endl;
		m_SocketServer->SendMSG(m_CurrentGlobalMsg);
	}
}

void HMI_Bridge::MainLoop()
{
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		// Receive the latest message send from socket based HMI
		if(m_SocketServer != nullptr)
		{
			PlannerHNS::HMI_MSG inc_msg;
			int bNew = m_SocketServer->GetLatestMSG(inc_msg);
			if(bNew > 0)
			{
				autoware_msgs::State state;
				state.mission_state = inc_msg.CreateStringMessage();
				//std::cout << "Received Data From HMI socket Client: " << state.mission_state << std::endl;
				pub_global_planner.publish(state);
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

}
