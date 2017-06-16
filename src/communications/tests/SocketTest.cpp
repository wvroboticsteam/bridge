#include "../SocketServer.hpp"
#include "../SocketClient.hpp"
#include "../../core/Thread.hpp"
#include <unistd.h>
#include <stdio.h>

using namespace SystemToolkit::Communications;
using namespace SystemToolkit::Core;

void* LaunchServer(void*);
void* LaunchClient(void*);

int port;

int main(int argc, char **argv)
{
	if(argc > 1)
		port = atoi(argv[1]);
	else
		port = 5000;

	try
	{
		StaticThread server(&LaunchServer, NULL);
		StaticThread client(&LaunchClient, NULL);

		server.LaunchThread();
		usleep(50000);
		client.LaunchThread();

		server.JoinThread();
		client.JoinThread();
	}
	catch(SystemToolkit::Communications::ERROR_TYPES e)
	{
		printf("caught comm error: %d\n", e);
	}
	catch(int e)
	{
		printf("caught system error: %X,%d\n", e, ExtractSystemError(e));
	}
	catch(...)
	{
		printf("wtf\n");
	}


	printf("Cleaning up\n");
	return 0;
}

void* LaunchServer(void*)
{
	SocketServer *socket = NULL;

	try
	{
		socket = new SocketServer(SOCKET_TYPE_TCP, port);
		
		if(!socket->ValidateParameters())
			printf("Invalid something or other\n");
		
		printf("[server] Connecting\n");
		socket->Connect();

		usleep(500000);		
	}
	catch(SystemToolkit::Communications::ERROR_TYPES e)
	{
		printf("[server] error: %X\n", e);
	}
	catch(int e)
	{
		printf("[server] error: %X\n", e);
	}

	
	delete socket;

	return NULL;
}

void* LaunchClient(void*)
{
	SocketClient *socket = NULL;

	try
	{
		socket = new SocketClient(SOCKET_TYPE_TCP, "127.0.0.1", port);

		if(!socket->ValidateParameters())
			printf("Invalid something or other\n");

		printf("[client] Connecting\n");
		socket->Connect();

		usleep(50000);

		socket->SendMessage("the quick brown fox ... oh you know", 35);

		usleep(50000);

		//socket->SendMessage("server-exit", 11);

		socket->Close();
	}
	catch(SystemToolkit::Communications::ERROR_TYPES e)
	{
		printf("Client error: %d\n", e);
	}
	catch(int e)
	{
		printf("Client error: %d\n", e);
	}

	delete socket;

	return NULL;
}
