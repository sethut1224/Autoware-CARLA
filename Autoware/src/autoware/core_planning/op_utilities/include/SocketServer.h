/*
 * SocketServer.h
 *
 *  Created on: Feb 13, 2017
 *      Author: user
 */

#ifndef SOCKETSERVER_H_
#define SOCKETSERVER_H_


#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include "op_planner/hmi/HMIMSG.h"


namespace HMI_BRIDGE_NS
{
class HMISocketServer
{
public:
	HMISocketServer();
	virtual ~HMISocketServer();
	int InitSocket(int port_send = 10001, int port_receive = 10002);
	void SendMSG(PlannerHNS::HMI_MSG msg);
	int GetLatestMSG(PlannerHNS::HMI_MSG& msg);

private:
	int m_Socket_send;
	int m_Socket_receive;
	pthread_mutex_t sock_mutex_send;
	pthread_t sock_thread_tid_send;
	PlannerHNS::HMI_MSG m_msg_send;
	bool m_bLatestMsg_send;
	pthread_mutex_t sock_mutex_receive;
	pthread_t sock_thread_tid_receive;
	PlannerHNS::HMI_MSG m_msg_receive;
	bool m_bLatestMsg_receive;

	//socklen_t m_Len;
	bool m_bExitMainLoop;

	int CreateSocket(int port, int& socket);

	static void* ThreadMainSend(void* pSock);
	static void* ThreadMainReceive(void* pSock);

};

}

#endif /* SOCKETSERVER_H_ */
