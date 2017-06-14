#ifndef __THREAD_HPP__
#define __THREAD_HPP__

#include "../CoreTypes.hpp"
#include <pthread.h>
#include <errno.h>

namespace SystemToolkit
{

namespace Core
{
    template<class T>
    class Thread
    {
    public:
	Thread();
	Thread(T*, void* (T::*)(void*), void*);
	Thread(const Thread&);
	virtual ~Thread();

	Thread& operator=(const Thread&);

	int SetInstance(T*);
	int SetData(void*);
	int SetTarget(void* (T::*)(void*));

	int LaunchThread();
	int JoinThread();

	bool GetJoinable();

    protected:
	static void* LaunchTarget(void*);
	void CopyObject(const Thread&);
	void SetRef();

	T *classInstance;
	void* (T::*threadTarget)(void*);
	void* threadData;
	bool threadJoined;

	pthread_t thread;
	int *refCount;
	bool isJoinable;
    };

    template<class T>
    Thread<T>::Thread()
    :classInstance(NULL)
    ,threadTarget(NULL)
    ,threadData(NULL)
    ,threadJoined(true)
    ,refCount(NULL)
    ,isJoinable(true)
    {
	SetRef();
    }

    template<class T>
    Thread<T>::Thread(T *instance, void* (T::*func)(void*), void *data)
    :classInstance(instance)
    ,threadTarget(func)
    ,threadData(data)
    ,threadJoined(true)
    ,refCount(NULL)
    ,isJoinable(true)
    {
	SetRef();
    }

    template<class T>
    Thread<T>::Thread(const Thread<T> &other)
    :classInstance(NULL)
    ,threadTarget(NULL)
    ,threadData(NULL)
    ,threadJoined(true)
    ,refCount(NULL)
    ,isJoinable(true)
    {
	CopyObject(other);
    }

    template<class T>
    Thread<T>::~Thread()
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

    template<class T>
    Thread<T>& Thread<T>::operator=(const Thread<T> &other)
    {
	JoinThread();
	CopyObject(other);
	return *this;
    }

    template<class T>
    int Thread<T>::SetInstance(T *instance)
    {
	if(threadJoined)
	{
	    classInstance = instance;
	    return ERROR_NONE;
	}
	return ERROR_RESOURCE_BUSY;
    }

    template<class T>
    int Thread<T>::SetData(void *data)
    {
	if(threadJoined)
	{
	    threadData = data;
	    return ERROR_NONE;
	}
	return ERROR_RESOURCE_BUSY;
    }

    template<class T>
    int Thread<T>::SetTarget(void* (T::*func)(void*))
    {
	if(threadJoined)
	{
	    threadTarget = func;
	    return ERROR_NONE;
	}
	return ERROR_RESOURCE_BUSY;
    }

    template<class T>
    int Thread<T>::LaunchThread()
    {
	int ret = ERROR_NONE;

	if((threadTarget == NULL) || (classInstance == NULL))
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

    template<class T>
    void* Thread<T>::LaunchTarget(void *threadData)
    {
	Thread<T> *parent = (Thread<T>*)threadData;
	return ((parent->classInstance)->*(parent->threadTarget))(parent->threadData);
    }

    template<class T>
    int Thread<T>::JoinThread()
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

    template<class T>
    void Thread<T>::SetRef()
    {
	if(refCount == NULL)
	{
	    refCount = new int;
	    (*refCount) = 0;
	}

	(*refCount)++;
    }

    template<class T>
    void Thread<T>::CopyObject(const Thread<T> &other)
    {
	classInstance = other.classInstance;
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

#endif
