/*
 * SocketServer.cpp
 *
 *  Created on: Feb 13, 2017
 *      Author: user
 */

#include "SocketServer.h"
#include <string.h>

namespace HMI_BRIDGE_NS
{
HMISocketServer::HMISocketServer()
{
	m_Socket_send = 0;
	m_Socket_receive = 0;
	sock_mutex_send =  PTHREAD_MUTEX_INITIALIZER;
	sock_mutex_receive =  PTHREAD_MUTEX_INITIALIZER;
	sock_thread_tid_send = 0;
	sock_thread_tid_receive = 0;
	m_bLatestMsg_send = false;
	m_bLatestMsg_receive = false;
	m_bExitMainLoop = false;
}

HMISocketServer::~HMISocketServer()
{
	std::cout << " >> Call The Destructor !!!!! " << std::endl;
	HMISocketServer* pRet;

	m_bExitMainLoop = true;
	shutdown(m_Socket_receive, SHUT_RDWR);
	if(sock_thread_tid_receive>0)
	{
		pthread_join(sock_thread_tid_receive, (void**)&pRet);
	}
	close(m_Socket_receive);

	shutdown(m_Socket_send, SHUT_RDWR);
	if(sock_thread_tid_send>0)
	{
		pthread_join(sock_thread_tid_send, (void**)&pRet);
	}
	close(m_Socket_send);

	std::cout << " >> Destroy everything !!!!! " << std::endl;

}

int HMISocketServer::CreateSocket(int port, int& socket_id)
{
	int err = -1;
	int reuse = 1;

	socket_id = socket(AF_INET, SOCK_STREAM, 0);
	if(socket_id == -1)
	{
		std::cout << "Error: Can't Create Socket." << std::endl;
		return -1;
	}

	err = setsockopt(socket_id, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
	if(err == -1)
	{
		shutdown(socket_id, SHUT_RDWR);
		close(socket_id);
		std::cout << "Error: Can't Set Socket Options." << std::endl;
		return -1;
	}

	sockaddr_in addr;
	memset(&addr, 0, sizeof(sockaddr_in));
	addr.sin_family = PF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = INADDR_ANY;

	err = bind(socket_id, (struct sockaddr *)&addr, sizeof(addr));
	if(err == -1)
	{
		shutdown(socket_id, SHUT_RDWR);
		close(socket_id);
		std::cout << "Error: Can't Bind Socket Address." << std::endl;
		return -1;
	}

	err = listen(socket_id, 5 /*maximum two clients can connect*/);
	if(err == -1)
	{
		std::cout << "Error: Can't Initialize Socket Listen Server." << std::endl;
		return -1;
	}

	return 1;
}

int HMISocketServer::InitSocket(int port_send, int port_receive)
{
	if(CreateSocket(port_send, m_Socket_send) == -1)
	{
		return -1;
	}

	if(pthread_create(&sock_thread_tid_send, nullptr,&HMISocketServer::ThreadMainSend , this) != 0)
	{
		std::cout << "Error: Can't Create Thread for Sending Socket ." << std::endl;
		return -1;
	}

	if(CreateSocket(port_receive, m_Socket_receive) == -1)
	{
		return -1;
	}

	if(pthread_create(&sock_thread_tid_receive, nullptr,&HMISocketServer::ThreadMainReceive , this) != 0)
	{
		std::cout << "Error: Can't Create Thread for Receive Socket ." << std::endl;
		return -1;
	}

	return 1;
}

void* HMISocketServer::ThreadMainSend(void* pSock)
{
	HMISocketServer* pS = (HMISocketServer*)pSock;

	while(!pS->m_bExitMainLoop)
	{
		int client_sock = 0;
		sockaddr_in client;
		socklen_t len = sizeof(client);
		client_sock = accept(pS->m_Socket_send, reinterpret_cast<sockaddr*>(&client), &len);
		if(client_sock == -1)
		{
			std::cout << "Can't Accept sending socket connection." << std::endl;
			usleep(100);
			continue;
		}

		bool bSkip = false;
		PlannerHNS::HMI_MSG msg;
		pthread_mutex_lock(&pS->sock_mutex_send);

		if(pS->m_bLatestMsg_send == false) // No Message to send , don't create any connections
		{
			bSkip = true;
		}
		else
		{
			msg = pS->m_msg_send;
			pS->m_bLatestMsg_send = false;
		}
		pthread_mutex_unlock(&pS->sock_mutex_send);

		if(!bSkip)
		{
			std::string cmd = msg.CreateStringMessage();
			ssize_t n = write(client_sock, cmd.c_str(), cmd.size());
			if(n < 0)
			{
				std::cout << "Can't Write message to socket." << std::endl;
				usleep(100);
				continue;
			}
		}


		shutdown(client_sock, SHUT_RDWR);
		if(close(client_sock) == -1)
		{
			std::cout << "Can't Close Send socket." << std::endl;
			usleep(100);
			continue;
		}
	}

	return 0;
}

void* HMISocketServer::ThreadMainReceive(void* pSock)
{
	HMISocketServer* pS = (HMISocketServer*)pSock;

	while(!pS->m_bExitMainLoop)
	{
		int client_sock = 0;
		sockaddr_in client;
		socklen_t len = sizeof(client);
		client_sock = accept(pS->m_Socket_receive, reinterpret_cast<sockaddr*>(&client), &len);
		if(client_sock == -1)
		{
			std::cout << "Can't Accept receiving socket connection." << std::endl;
			usleep(100);
			continue;
		}

		char recvdata[1024];
		std::string can_data("");
		ssize_t nr = 0;
		while(true)
		{
			nr = recv(client_sock, recvdata, sizeof(recvdata), 0);
			if(nr<0)
			{
				std::cout << "Can't receive message from socket." << std::endl;
				can_data = "";
				break;
			}
			else if(nr == 0)
			{
				break;
			}

			can_data.append(recvdata,nr);
		}

		shutdown(client_sock, SHUT_RDWR);
		if(close(client_sock)<0)
		{
			std::cout << "Can't Close receive socket." << std::endl;
			usleep(100);
			continue;
		}

		if(can_data.size() > 0)
		{
			pthread_mutex_lock(&pS->sock_mutex_receive);
			PlannerHNS::HMI_MSG msg_temp = PlannerHNS::HMI_MSG::FromString(can_data);
			if(msg_temp.type != PlannerHNS::UNKNOWN_MSG)
			{
				pS->m_bLatestMsg_receive = true;
				pS->m_msg_receive = msg_temp;
			}
			pthread_mutex_unlock(&pS->sock_mutex_receive);
		}
	}

	return 0;
}

void HMISocketServer::SendMSG(PlannerHNS::HMI_MSG msg)
{
	pthread_mutex_lock(&sock_mutex_send);
	m_bLatestMsg_send = true;
	m_msg_send = msg;
	pthread_mutex_unlock(&sock_mutex_send);
}

int HMISocketServer::GetLatestMSG(PlannerHNS::HMI_MSG& msg)
{
	int res = -1;
	pthread_mutex_lock(&sock_mutex_receive);
	if(m_bLatestMsg_receive == true)
	{
		msg = m_msg_receive;
		m_bLatestMsg_receive = false;
		res = 1;
	}
	pthread_mutex_unlock(&sock_mutex_receive);

	return res;
}

}



