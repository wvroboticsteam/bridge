#include "StaticThread.hpp"

namespace SystemToolkit
{

namespace Core
{
    StaticThread::StaticThread()
    :threadTarget(NULL)
    ,threadData(NULL)
    ,threadJoined(true)
    ,refCount(NULL)
    ,isJoinable(true)
    {
	SetRef();
    }

    StaticThread::StaticThread(void* (*func)(void*), void *data)
    :threadTarget(func)
    ,threadData(data)
    ,threadJoined(true)
    ,refCount(NULL)
    ,isJoinable(true)
    {
	SetRef();
    }

    StaticThread::StaticThread(const StaticThread &other)
    :threadTarget(NULL)
    ,threadData(NULL)
    ,threadJoined(true)
    ,refCount(NULL)
    ,isJoinable(true)
    {
	CopyObject(other);
    }

    StaticThread::~StaticThread()
    {
	if(refCount != NULL)
	{
	    (*refCount)--;
	    if((*refCount) <= 0)
	    {
		JoinThread();
		delete refCount;
		refCount = NULL;
	    }
	}
    }

    StaticThread& StaticThread::operator=(const StaticThread &other)
    {
	JoinThread();
	CopyObject(other);
	return *this;
    }

    int StaticThread::DetachThread()
    {
	int ret;

	if(isJoinable)
	    ret = pthread_detach(pthread_self());
	else
	    ret = ERROR_ILLEGAL_REQUEST;

	if(ret == 0)
	{
	    isJoinable = false;
	    return ERROR_NONE;
	}
	
	return (ERROR_SYSTEM_SPECIFIC | ret);
    }

    int StaticThread::SetData(void *data)
    {
	if(threadJoined)
	{
	    threadData = data;
	    return ERROR_NONE;
	}
	return ERROR_RESOURCE_BUSY;
    }

    int StaticThread::SetTarget(void* (*func)(void*))
    {
	if(threadJoined)
	{
	    threadTarget = func;
	    return ERROR_NONE;
	}
	return ERROR_RESOURCE_BUSY;
    }

    int StaticThread::LaunchThread()
    {
	int ret = ERROR_NONE;

	if(threadTarget == NULL)
	    ret = ERROR_INVALID_PARAMETER;
	else
	{
	    ret = pthread_create(&thread, NULL, LaunchTarget, this);
	    if(ret == 0)
		threadJoined = false;
	    else
		ret = GenerateSystemDescriptiveError(errno);
	}

	return ret;
    }

    void* StaticThread::LaunchTarget(void *threadData)
    {
	StaticThread *parent = (StaticThread*)threadData;
	return (*(parent->threadTarget))(parent->threadData);
    }

    int StaticThread::JoinThread()
    {
	int ret = 0;

	if(threadJoined)
	    ret = ERROR_NONE;
	else if((ret = pthread_join(thread, NULL)) == 0)
	    threadJoined = true;
	else
	    ret = GenerateSystemDescriptiveError(errno);

	return ret;
    }

    void StaticThread::SetRef()
    {
	if(refCount == NULL)
	{
	    refCount = new int;
	    (*refCount) = 0;
	}

	(*refCount)++;
    }

    void StaticThread::CopyObject(const StaticThread &other)
    {
	threadTarget = other.threadTarget;
	threadData = other.threadData;
	threadJoined = other.threadJoined;
	thread = other.thread;
	isJoinable = other.isJoinable;

	if(refCount != NULL)
	{
	    if((*refCount) == 1)
		delete refCount;
	    else
		(*refCount)--;
	}

	refCount = other.refCount;

	if(refCount != NULL)
	    (*refCount)++;
    }

}

}
