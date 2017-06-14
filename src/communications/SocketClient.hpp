#ifndef __SOCKETCLIENT_HPP__
#define __SOCKETCLIENT_HPP__

#include "Socket.hpp"
#include "../core/Thread.hpp"

namespace SystemToolkit
{

namespace Communications
{

	class SocketClient : public Socket
	{
	public:
		SocketClient();
		SocketClient(SOCKET_TYPE, const char*, int);
		SocketClient(const SocketClient&);
		~SocketClient();

		SocketClient& operator=(const SocketClient&);
		void Connect();
		void Close();

		void SendMessage(const char*, size_t);

	protected:
		void CopyObject(const SocketClient&);
		void SetRef();
		void* ReadThread(void*);

		int *refCount;
		bool loopControl;
		Core::Thread<SocketClient> *readThread;
		char buffer[1024];
		int socketFD;
	};

}

}

#endif
