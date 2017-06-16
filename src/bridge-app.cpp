#include "RosBridge.hpp"
#include <signal.h>
#include "communications/SocketServer.hpp"

void SignalHandler(int);
void ServerCallback(void*, unsigned int);

static bool run = true;

using namespace SystemToolkit::Core;
using namespace SystemToolkit::Communications;

int main()
{
    RosBridge *bridge = new RosBridge;
    signal(SIGINT, SignalHandler);

    if(!bridge->RunLoopThread(10000000)) //100 Hz
    {
	ROS_ERROR("Failed to launch ros thread\n");
	return -1;
    }

    SocketServer *socket = NULL;

    try
    {
	socket = new SocketServer(SOCKET_TYPE_UDP, 8001);
		
	if(!socket->ValidateParameters())
	    printf("Invalid something or other\n");
		
	printf("[server] Connecting\n");
	socket->Connect();
    }
    catch(SystemToolkit::Communications::ERROR_TYPES e)
    {
	printf("[server] error: %X\n", e);
    }
    catch(int e)
    {
	printf("[server] error: %X\n", e);
    }
	
    while(run)
	usleep(50000);

    delete socket;
    delete bridge;
    return 0;
}

void SignalHandler(int)
{
    run = false;
}

void ServerCallback(void *data, unsigned int dataSize)
{
    unsigned int command;

    if(dataSize == 4)
    {
	memcpy(&command, data, 4);
	
	switch(command)
	{
	    case 0:
		
	}
    }
}
