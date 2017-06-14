#ifndef __SOCKETSERVER_HPP__
#define __SOCKETSERVER_HPP__

#include "Socket.hpp"
#include "../core/Thread.hpp"
#include "../types/Array.hpp"

namespace SystemToolkit
{

namespace Communications
{

	class SocketServer : public Socket
	{
	public:
		SocketServer();
		SocketServer(SOCKET_TYPE, int);
		SocketServer(const SocketServer&);
		~SocketServer();

		SocketServer& operator=(const SocketServer&);
		void Connect();
		void Close();

	protected:
		struct AcceptThreadData
		{
			SocketServer *parent;
			int socketFD;

			AcceptThreadData()
			:parent(NULL)
			,socketFD(0)
				{

				}

			AcceptThreadData(SocketServer *instance, int fd)
			:parent(instance)
			,socketFD(fd)
				{

				}

			AcceptThreadData& operator=(const AcceptThreadData &other)
				{
					parent = other.parent;
					socketFD = other.socketFD;
					return *this;
				}
		};

		struct ReadThreadData
		{
			Core::Thread<SocketServer> thread;
			SocketServer *parent;
			bool connected;
			char readBuffer[1024];
			int readFD;
			int exitCode;

			ReadThreadData()
			:connected(false)
			,readFD(-1)
			,exitCode(0)
				{

				}

			void ClearBuffer()
				{
					memset(readBuffer, 0, 1024);
				}
		};

		void CopyObject(const SocketServer&);
		void SetRef();
		void* AcceptConnectionThread(void*);
		void* ReadThread(void*);
		void SpawnReadThread(int);

		int *refCount;
		Core::Thread<SocketServer> *acceptThread;
		AcceptThreadData acceptThreadData;
		Types::Array<ReadThreadData> *readThreadData;
		int acceptThreadReturn;
		bool acceptingConnections;
	};

}

}

#endif
