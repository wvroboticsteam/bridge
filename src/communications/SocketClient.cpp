#include "SocketClient.hpp"
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <poll.h>
#include <stdio.h>

using namespace SystemToolkit::Core;

namespace SystemToolkit
{

namespace Communications
{

    SocketClient::SocketClient()
    :Socket(SOCKET_DIR_CLIENT, SOCKET_TYPE_UNKNOWN)
    ,refCount(NULL)
    ,loopControl(false)
    ,readThread(NULL)
    {
	SetRef();
    }

    SocketClient::SocketClient(SOCKET_TYPE type, const char *addr, int port)
    :Socket(SOCKET_DIR_CLIENT, type, addr, port)
    ,refCount(NULL)
    ,loopControl(false)
    ,readThread(NULL)
    {
	SetRef();
    }

    SocketClient::SocketClient(const SocketClient &other)
    :Socket(other)
    ,refCount(NULL)
    ,loopControl(false)
    ,readThread(NULL)
    {
	CopyObject(other);
    }

    SocketClient::~SocketClient()
    {
	if(refCount != NULL)
	{
	    (*refCount)--;
	    if((*refCount) <= 0)
	    {
		delete refCount;
		refCount = NULL;

		if(readThread != NULL)
		{
		    loopControl = false;
		    readThread->JoinThread();
		    delete readThread;
		    readThread = NULL;
		}
	    }
	    Close();
	}
    }

    SocketClient& SocketClient::operator=(const SocketClient &other)
    {
	CopyObject(other);
	return *this;
    }

    void SocketClient::Connect()
    {
	int type;
	struct sockaddr_in serverAddress;
	struct hostent *server;

	if(socketOpen)
	    return;
			
	SetSocketType(type);
			
	socketFD = socket(AF_INET, type, IPPROTO_IP);
	if(socketFD < 0)
	    throw GenerateSystemDescriptiveError(errno);
			
	server = gethostbyname(socketAddress);
	if(server == NULL)
	{
	    close(socketFD);
	    throw ERROR_NO_SERVER;
	}
			
	memset(&serverAddress, 0, sizeof(serverAddress));
	serverAddress.sin_family = AF_INET;
	memcpy(&serverAddress.sin_addr.s_addr, server->h_addr, server->h_length);
	serverAddress.sin_port = htons(socketPort);
			
	if(connect(socketFD, (struct sockaddr*) &serverAddress, sizeof(serverAddress)) < 0)
	{
	    close(socketFD);
	    throw ERROR_NO_SERVER;
	}
	socketOpen = true;

	//SendMessage("Hello From Client", 17);

	loopControl = true;
	readThread = new Thread<SocketClient>(this, &SocketClient::ReadThread, this);
	memset(buffer, 0, 1024);
	readThread->LaunchThread();
    }

    void SocketClient::SendMessage(const void *msg, size_t msgSize)
    {
	if(socketOpen)
	{
	    if(write(socketFD, msg, msgSize) < 0)
		throw GenerateSystemDescriptiveError(errno);
	}
	else
	    throw ERROR_NOT_READY;
    }

    void SocketClient::Close()
    {
	shutdown(socketFD, SHUT_RDWR);
	close(socketFD);
    }

    void SocketClient::CopyObject(const SocketClient &other)
    {
	if(refCount != NULL)
	{
	    if((*refCount) == 1)
		delete refCount;
	    else
		(*refCount)--;
	}

	loopControl = other.loopControl;
	readThread = other.readThread;
	refCount = other.refCount;

	if(refCount != NULL)
	    (*refCount)++;
    }

    void SocketClient::SetRef()
    {
	if(refCount == NULL)
	{
	    refCount = new int;
	    (*refCount) = 0;
	}

	(*refCount)++;
    }

    void* SocketClient::ReadThread(void *data)
    {
	SocketClient *parent = (SocketClient*)data;
	ssize_t bytesRead;
	struct pollfd pollFD;
	pollFD.fd = parent->socketFD;
	pollFD.events = POLLIN | POLLPRI;

	while(parent->loopControl)
	{
	    pollFD.revents = 0;
	    poll(&pollFD, 1, 5000);
	    if((pollFD.revents & POLLRDHUP) || (pollFD.revents & POLLHUP))
	    {
		break;
	    }
	    else if((pollFD.revents & POLLIN) || (pollFD.revents & POLLPRI))
	    {
		if((bytesRead = read(parent->socketFD, parent->buffer, 1024)) > 0)
		{
		    if(bytesRead < 1022)
			parent->buffer[bytesRead+1] = 0;
		    else
			parent->buffer[1023] = 0;

		    if(callback)
			parent->callback(parent->buffer, bytesRead, parent->socketFD);
		    else
			printf("Client Received MSG: %s\n", parent->buffer);
		}
		else if(bytesRead < 0)
		{
		    //throw GenerateSystemDescriptiveError(errno);
		    printf("ReadThread [client] caught error: %ld, %d, %X\n", (long int)bytesRead, errno, pollFD.revents);
		}
	    }
	}

	close(parent->socketFD);
	return NULL;
    }

}

}
