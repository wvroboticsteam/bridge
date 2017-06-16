#include "SocketServer.hpp"
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>
#include <malloc.h>
#include <errno.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <poll.h>

using namespace SystemToolkit::Core;
using namespace SystemToolkit::Types;

namespace SystemToolkit
{

namespace Communications
{

    SocketServer::SocketServer()
    :Socket(SOCKET_DIR_SERVER, SOCKET_TYPE_UNKNOWN)
    ,refCount(NULL)
    ,acceptThread(NULL)
    ,readThreadData(NULL)
    {
	SetRef();
    }

    SocketServer::SocketServer(SOCKET_TYPE type, int port)
    :Socket(SOCKET_DIR_SERVER, type, "127.0.0.1", port)
    ,refCount(NULL)
    ,acceptThread(NULL)
    ,readThreadData(NULL)
    {
	SetRef();
    }
	
    SocketServer::SocketServer(const SocketServer &other)
    :Socket(other)
    {
	CopyObject(other);
    }
	
    SocketServer::~SocketServer()
    {
	if(refCount != NULL)
	{
	    (*refCount)--;
	    if((*refCount) <= 0)
	    {
		Close();

		free(refCount);
		refCount = NULL;

		if(acceptThread != NULL)
		{
		    loopControl = false;
		    acceptThread->JoinThread();
		    delete acceptThread;
		}

		if(readThreadData != NULL)
		{
		    for(size_t i=0; i<readThreadData->GetSize(); i++)
			readThreadData->operator[](i).thread.JoinThread();

		    delete readThreadData;
		    readThreadData = NULL;
		}
	    }
	}
    }

    SocketServer& SocketServer::operator=(const SocketServer &other)
    {
	CopyObject(other);
	return *this;
    }

    void SocketServer::Connect()
    {
	int socketFD = -1;
	struct sockaddr_in serverAddress;
	int type;
	ReadThreadData sampleData;

	if(!ValidateParameters())
	    throw ERROR_INVALID_PARAMETER;

	socketOpen = true;
	SetSocketType(type);

	memset(&serverAddress, 0, sizeof(serverAddress));
	if((socketFD = socket(AF_INET, type, IPPROTO_IP)) == -1)
	    goto CON_SER_FAIL;

	serverAddress.sin_family = AF_INET;
	serverAddress.sin_addr.s_addr = INADDR_ANY;
	serverAddress.sin_port = htons(socketPort);

	if(bind(socketFD, (struct sockaddr*) &serverAddress, sizeof(serverAddress)) < 0)
	    goto CON_SER_FAIL;

	if(listen(socketFD, serverBacklog) < 0)
	    goto CON_SER_FAIL;

	loopControl = true;

	acceptThreadData.parent = this;
	acceptThreadData.socketFD = socketFD;

	sampleData.parent = this;
	sampleData.connected = false;
	readThreadData = new Array<ReadThreadData>(serverBacklog, sampleData, false);
	acceptThread = new Thread<SocketServer>(this, &SocketServer::AcceptConnectionThread, &acceptThreadData);
	acceptingConnections = true;
	acceptThread->LaunchThread();

	return;

    CON_SER_FAIL:
	if(socketFD > -1)
	    close(socketFD);

	socketOpen = false;
	throw GenerateSystemDescriptiveError(errno);
    }

    void SocketServer::SpawnReadThread(int readFD)
    {
	bool launched = false;
	for(size_t i=0; i<readThreadData->GetSize(); i++)
	{
	    if(!readThreadData->operator[](i).connected)
	    {
		readThreadData->operator[](i).ClearBuffer();
		readThreadData->operator[](i).connected = true;
		readThreadData->operator[](i).parent = this;
		readThreadData->operator[](i).readFD = readFD;
		readThreadData->operator[](i).thread = Thread<SocketServer>(this, &SocketServer::ReadThread, &(readThreadData->operator[](i)));
		readThreadData->operator[](i).thread.LaunchThread();

		launched = true;
		break;
	    }
	}

	if(!launched)
	    throw ERROR_LACK_RESOURCES;
    }

    bool SocketServer::SendMessage(const void *data, unsigned int dataSize, int fd)
    {
	return (write(fd, data, dataSize) == dataSize);
    }

    void* SocketServer::AcceptConnectionThread(void *data)
    {
	AcceptThreadData *threadData = (AcceptThreadData*)data;
	SocketServer *parent = threadData->parent;

	int socketFD = threadData->socketFD;
	struct sockaddr_in clientAddress;
	socklen_t clientLength = sizeof(clientAddress);
	int newSocketFD;
	int errorCount = 0;
	parent->acceptThreadReturn = ERROR_NONE;

	while(parent->loopControl)
	{
	    newSocketFD = accept(socketFD, (struct sockaddr*) &clientAddress, &clientLength);
	    if(newSocketFD < 0)
		parent->loopControl = false;
	    else
	    {
		if(write(newSocketFD, "Server ACK", 10) < 0)
		    parent->loopControl = false;
		else
		{
		    try
		    {
			parent->SpawnReadThread(newSocketFD);
		    }
		    catch(ERROR_TYPES e)
		    {
			parent->acceptThreadReturn = e;
			if(errorCount++ > 5)
			    break;
		    }
		}
	    }
	}

	parent->acceptingConnections = false;
	close(socketFD);
	return NULL;
    }

    void* SocketServer::ReadThread(void *data)
    {
	ReadThreadData *threadData = (ReadThreadData*)data;
	ssize_t bytesRead;
	struct pollfd pollFD;
	pollFD.fd = threadData->readFD;
	pollFD.events = POLLIN | POLLPRI;
	threadData->exitCode = ERROR_NONE;

	while(threadData->parent->loopControl)
	{
	    pollFD.revents = 0;
	    poll(&pollFD, 1, 5000);
	    if((pollFD.revents & POLLIN) || (pollFD.revents & POLLPRI))
	    {
		threadData->ClearBuffer();
		if((bytesRead = read(threadData->readFD, threadData->readBuffer, 1024)) > 0)
		{
		    if(bytesRead < 1022)
			threadData->readBuffer[bytesRead+1] = 0;
		    else
			threadData->readBuffer[1023] = 0;
			
		    if(threadData->parent->callback)
			threadData->parent->callback(threadData->readBuffer, bytesRead, threadData->readFD);

		    printf("Server [%d] Received MSG: %s\n", threadData->readFD, threadData->readBuffer);
		}
		else if(bytesRead < 0)
		{
		    threadData->exitCode = GenerateSystemDescriptiveError(errno);
		    break;
		}
	    }
	    else if(pollFD.revents & POLLRDHUP)
	    {
		break;
	    }
	}
	usleep(100000);
	close(threadData->readFD);
	threadData->connected = false;
	return NULL;
    }

    void SocketServer::Close()
    {
	if(socketOpen)
	{
			
	    if(loopControl && (socketDirection == SOCKET_DIR_SERVER))
	    {
		loopControl = false;
				
		int socketFD, type;
		struct sockaddr_in serverAddress;
		struct hostent *server;

		SetSocketType(type);

		socketFD = socket(AF_INET, type, IPPROTO_IP);
		if(socketFD < 0)
		    throw GenerateSystemDescriptiveError(errno);

		server = gethostbyname("127.0.0.1");
		if(server == NULL)
		{
		    close(socketFD);
		    throw ERROR_INCORRECT_LOOPBACK;
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
		write(socketFD, "server-exit", 11);
		close(socketFD);
	    }

	    socketOpen = false;
	}
    }

    void SocketServer::SetRef()
    {
	if(refCount == NULL)
	{
	    if((refCount = (int*)malloc(sizeof(int))) == NULL)
		throw GenerateSystemDescriptiveError(errno);

	    *refCount = 0;
	}

	(*refCount)++;
    }

    void SocketServer::CopyObject(const SocketServer &other)
    {
	this->~SocketServer();

	refCount = other.refCount;
	acceptThread = other.acceptThread;
	acceptThreadData = other.acceptThreadData;
	readThreadData = other.readThreadData;
	acceptThreadReturn = other.acceptThreadReturn;
	acceptingConnections = other.acceptingConnections;
	callback = other.callback;

	if(refCount != NULL)
	    (*refCount)++;
    }

}

}
