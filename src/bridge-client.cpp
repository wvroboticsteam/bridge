#include "communications/SocketClient.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

using namespace SystemToolkit::Communications;

bool wait=false;

void Callback(void*, unsigned int, int);

int main(int argc, char *argv[])
{
    if(argc != 3)
    {
	printf("Call this program like: ./bridge-client ip cmd\nwhere ip is the server ip address and cmd is the command code\n");
	return 1;
    }

    unsigned int command = atol(argv[2]);

    try
    {
	SocketClient client(SOCKET_TYPE_UDP, argv[1], 8001);
	if(!client.ValidateParameters())
	{
	    printf("Invalid something or other\n");
	    return 2;
	}

	printf("[client] Connecting\n");
	client.Connect();

	usleep(50000);
	client.SendMessage(&command, sizeof(unsigned int));
	wait = true;
    }
    catch(SystemToolkit::Communications::ERROR_TYPES e)
    {
	printf("Client error: %d\n", e);
    }
    catch(int e)
    {
	printf("Client error: %d\n", e);
    }

    int limit=0;
    while(wait)
    {
	if(limit++ > 60)
	    break;
	sleep(1);
    }
}

void Callback(void *data, unsigned int dataSize, int)
{
    double pelvis[3];

    if(dataSize == (sizeof(double) * 3))
    {
	memcpy(pelvis, data, sizeof(double) * 3);
	printf("Client received pelvis origin: %3.f, %.3f, %.3f\n",
	       pelvis[0], pelvis[1], pelvis[2]);
    }
    else
    {
	char *myString = new char[dataSize + 1];
	memcpy(myString, data, dataSize);
	myString[dataSize] = 0;

	printf("Client received string: %s\n", myString);
	delete[] myString;
    }

    wait = false;
}
