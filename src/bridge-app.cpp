#include "RosBridge.hpp"
#include <signal.h>
#include "communications/SocketServer.hpp"

void SignalHandler(int);
void ServerCallback(void*, unsigned int, int);

static bool run = true;

using namespace SystemToolkit::Core;
using namespace SystemToolkit::Communications;

RosBridge *myBridge;
SocketServer *mySocket = NULL;

int main()
{
    myBridge = new RosBridge;
    signal(SIGINT, SignalHandler);

    if(!myBridge->RunLoopThread(10000000)) //100 Hz
    {
	ROS_ERROR("Failed to launch ros thread\n");
	return -1;
    }

    try
    {
	mySocket = new SocketServer(SOCKET_TYPE_UDP, 8001);
		
	if(!mySocket->ValidateParameters())
	    printf("Invalid something or other\n");

	mySocket->SetCallback(ServerCallback);
	printf("[server] Connecting\n");
	mySocket->Connect();
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

    delete mySocket;
    delete myBridge;
    return 0;
}

void SignalHandler(int)
{
    run = false;
}

void ServerCallback(void *data, unsigned int dataSize, int)
{
    unsigned int command;
    tf::StampedTransform pelvis;
    double result[3];
    tf::Vector3 vec;

    ROS_INFO("Server callback entered, data: %p, size: %u\n", data, dataSize);

    if(dataSize == 4)
    {
	memcpy(&command, data, 4);
	ROS_INFO("Data converted to unsigned int with value: %u\n", command);

	switch(command)
	{
	    case 0:
		if(!myBridge->BasicCommands(command))
		{
		    mySocket->UDPSendMessage("error", 5);
		    ROS_ERROR("BasicCommand of 0 failed\n");
		}
		else
		    mySocket->UDPSendMessage("ack", 3);
		break;
	    case 1:
		if(!myBridge->BasicCommands(command, &pelvis))
		{
		    mySocket->UDPSendMessage("error", 5);
		    ROS_ERROR("BasicCommand of 0 failed\n");
		}
		else
		{
		    vec = pelvis.getOrigin();

		    result[0] = vec.getX();
		    result[1] = vec.getY();
		    result[2] = vec.getZ();

		    ROS_INFO("Sending: %f, %f, %f", result[0], result[1], result[2]);
		    mySocket->UDPSendMessage(result, sizeof(double) * 3);
		}
		break;
	    default:
		ROS_ERROR("Invalid option received: %u\n", command);
		mySocket->UDPSendMessage("invalid", 7);
		break;
	}
    }
}
