#include "RosBridge.hpp"
#include <signal.h>

void SignalHandler(int);

static bool run = true;

int main()
{
    RosBridge *bridge = new RosBridge;
    signal(SIGINT, SignalHandler);

    if(!bridge->RunLoopThread(10000000)) //100 Hz
    {
	ROS_ERROR("Failed to launch ros thread\n");
	return -1;
    }

    while(run)
	usleep(50000);

    delete bridge;
    return 0;
}

void SignalHandler(int)
{
    run = false;
}
