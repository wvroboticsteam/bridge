#ifndef __STATICTHREAD_HPP__
#define __STATICTHREAD_HPP__

#include "../CoreTypes.hpp"
#include <pthread.h>
#include <errno.h>

namespace SystemToolkit
{

namespace Core
{

    class StaticThread
    {
    public:
	StaticThread();
	StaticThread(void* (*)(void*), void*);
	StaticThread(const StaticThread&);
	virtual ~StaticThread();

	StaticThread& operator=(const StaticThread&);

	int SetData(void*);
	int SetTarget(void* (*)(void*));

	int LaunchThread();	    
	int JoinThread();
	int DetachThread();

	bool GetJoinable();

    protected:
	static void* LaunchTarget(void*);
	void CopyObject(const StaticThread&);
	void SetRef();

	void* (*threadTarget)(void*);
	void* threadData;
	bool threadJoined;

	pthread_t thread;
	int *refCount;
	bool isJoinable;
    };

}

}

#endif
