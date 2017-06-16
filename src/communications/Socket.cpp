#include "Socket.hpp"
#include <malloc.h>
#include <arpa/inet.h>
#include <string.h>
#include <errno.h>

namespace SystemToolkit
{

namespace Communications
{

    Socket::Socket()
    :socketDirection(SOCKET_DIR_UNKNOWN)
    ,socketType(SOCKET_TYPE_UNKNOWN)
    ,socketPort(-1)
    ,socketAddress(NULL)
    ,socketOpen(false)
    ,serverBacklog(5)
    ,loopControl(false)
    ,callback(NULL)
    {

    }

    Socket::Socket(SOCKET_DIRECTION direction, SOCKET_TYPE type)
    :socketDirection(direction)
    ,socketType(type)
    ,socketPort(-1)
    ,socketAddress(NULL)
    ,socketOpen(false)
    ,serverBacklog(5)
    ,loopControl(false)
    ,callback(NULL)
    {

    }

    Socket::Socket(SOCKET_DIRECTION direction, SOCKET_TYPE type, const char *ip, int port)
    :socketDirection(direction)
    ,socketType(type)
    ,socketPort(port)
    ,socketAddress(NULL)
    ,socketOpen(false)
    ,serverBacklog(5)
    ,loopControl(false)
    ,callback(NULL)
    {
	SetAddress(ip);
    }

    Socket::Socket(const Socket &other)
    :socketDirection(SOCKET_DIR_UNKNOWN)
    ,socketType(SOCKET_TYPE_UNKNOWN)
    ,socketPort(-1)
    ,socketAddress(NULL)
    ,socketOpen(false)
    ,serverBacklog(5)
    ,loopControl(false)
    ,callback(NULL)
    {
	CopyObject(other);
    }
	
    Socket::~Socket()
    {
	if(socketAddress != NULL)
	    free(socketAddress);
    }

    void Socket::SetCallback(void (*func)(void*, unsigned int, int))
    {
	callback = func;
    }



    Socket& Socket::operator=(const Socket &other)
    {
	CopyObject(other);
	return *this;
    }

    bool Socket::ValidateParameters()
    {
	return ((socketDirection != SOCKET_DIR_UNKNOWN) &&
		(socketType != SOCKET_TYPE_UNKNOWN) &&
		(ValidatePort(socketPort)) &&
		(ValidateIP(socketAddress)));
    }

    void Socket::ConnectClient()
    {

    }

    void Socket::SetSocketType(int &type)
    {
	switch(socketType)
	{
	    case SOCKET_TYPE_TCP:
		type = SOCK_STREAM;
		break;
	    case SOCKET_TYPE_UDP:
		type = SOCK_DGRAM;
		break;
	    default:
		throw ERROR_INVALID_PARAMETER;
	}
    }

    void Socket::SetDirection(SOCKET_DIRECTION direction)
    {
	if(socketOpen)
	{
	    throw ERROR_PARAMETER_LOCKED;
	}

	socketDirection = direction;
    }

    void Socket::SetType(SOCKET_TYPE type)
    {
	if(socketOpen)
	{
	    throw ERROR_PARAMETER_LOCKED;
	}

	socketType = type;
    }

    void Socket::SetPort(int port)
    {
	if(socketOpen)
	{
	    throw ERROR_PARAMETER_LOCKED;
	}
		
	socketPort = port;
    }

    void Socket::SetAddress(const char *ip)
    {
	if(socketOpen)
	{
	    throw ERROR_PARAMETER_LOCKED;
	}
	else if(ValidateIP(ip))
	{
	    if(socketAddress != NULL)
		free(socketAddress);

	    socketAddress = (char*)malloc(sizeof(char) * (strlen(ip) + 1));
	    if(socketAddress != NULL)
		strcpy(socketAddress, ip);
	    else
		throw GenerateSystemDescriptiveError(errno);
	}
	else
	{
	    throw ERROR_INVALID_PARAMETER;
	}
    }

    void Socket::SetServerBacklog(int backlog)
    {
	if(socketOpen)
	{
	    throw ERROR_PARAMETER_LOCKED;
	}

	serverBacklog = backlog;
    }

    int Socket::GetServerBacklog() const
    {
	return serverBacklog;
    }

    SOCKET_DIRECTION Socket::GetDirection() const
    {
	return socketDirection;
    }

    SOCKET_TYPE Socket::GetType() const
    {
	return socketType;
    }

    int Socket::GetPort() const
    {
	return socketPort;
    }

    const char* Socket::GetAddress() const
    {
	return socketAddress;
    }

    bool Socket::GetOpen() const
    {
	return socketOpen;
    }

    bool Socket::ValidateIP(const char *ip)
    {
	char *ptr, *testIP;
	int octetCount=0;

	testIP = (char*)malloc((strlen(ip) + 1) * sizeof(char));

	if(testIP == NULL)
	    throw GenerateSystemDescriptiveError(errno);

	strcpy(testIP, ip);
		
	ptr = strtok(testIP, ".");
	while(ptr != NULL)
	{
	    if(strlen(ptr) > 3)
	    {
		free(testIP);
		return false;
	    }

	    for(size_t i=0; i<strlen(ptr); i++)
	    {
		if((ptr[i] < 48) || (ptr[i] > 57))
		{
		    free(testIP);
		    return false;
		}
	    }

	    octetCount++;
	    ptr = strtok(NULL, ".");
	}

	free(testIP);
	return (octetCount == 4);
    }

    bool Socket::ValidatePort(int port)
    {
	return ((port >= 0) && (port <= 65535));
    }

    void Socket::CopyObject(const Socket &other)
    {
	socketDirection = other.socketDirection;
	socketType = other.socketType;
	socketPort = other.socketPort;
	socketOpen = other.socketOpen;
	SetAddress(other.socketAddress);
	callback = other.callback;
    }

}

}
