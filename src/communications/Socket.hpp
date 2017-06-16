#ifndef __SOCKET_HPP__
#define __SOCKET_HPP__

#include "CommunicationsTypes.hpp"

namespace SystemToolkit
{

namespace Communications
{

    class Socket
    {
    public:
	Socket();
	Socket(SOCKET_DIRECTION, SOCKET_TYPE);
	Socket(SOCKET_DIRECTION, SOCKET_TYPE, const char*, int);
	Socket(const Socket&);
	virtual ~Socket();

	Socket& operator=(const Socket&);
		
	virtual void Connect() = 0;
	virtual void Close() = 0;
	virtual bool ValidateParameters();
	virtual bool ValidateIP(const char*);
	virtual bool ValidatePort(int);

	void SetDirection(SOCKET_DIRECTION);
	void SetType(SOCKET_TYPE);
	void SetPort(int);
	void SetAddress(const char*);
	void SetServerBacklog(int);
		
	SOCKET_DIRECTION GetDirection() const;
	SOCKET_TYPE GetType() const;
	int GetPort() const;
	const char* GetAddress() const;
	bool GetOpen() const;
	int GetServerBacklog() const;
	void SetCallback(void (*)(void*, unsigned int, int));
		
    protected:
	void ConnectServer();
	void ConnectClient();
	void SetSocketType(int&);
	void CopyObject(const Socket&);
		
	SOCKET_DIRECTION socketDirection;
	SOCKET_TYPE socketType;
	int socketPort;
	char *socketAddress;
	bool socketOpen;
	int serverBacklog;
	bool loopControl;
	void (*callback)(void*, unsigned int, int);
    };

}

}

#endif
