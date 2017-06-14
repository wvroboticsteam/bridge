#include "Mutex.hpp"
#include "../CoreTypes.hpp"
#include <errno.h>

namespace SystemToolkit
{

namespace Core
{
	
    Mutex::Mutex(bool init):mutex(NULL)
    {
	if(init)
	    Initialize();
    }

    Mutex::~Mutex()
    {
	Deallocate();
    }

    int Mutex::Initialize()
    {
	if(mutex != NULL)
	    return ERROR_INVALID_PARAMETER;

	if(pthread_mutex_init(mutex, NULL) != 0)
	{
	    mutex = NULL;
	    return GenerateSystemDescriptiveError(errno);
	}
	return ERROR_NONE;
    }

    int Mutex::Deallocate()
    {
	if(mutex == NULL)
	    return ERROR_INVALID_PARAMETER;

	if(pthread_mutex_destroy(mutex) != 0)
	    return GenerateSystemDescriptiveError(errno);

	mutex = NULL;
	return ERROR_NONE;
    }

    bool Mutex::Ready()
    {
	return (mutex != NULL);
    }
	
    int Mutex::Lock()
    {
	if(pthread_mutex_lock(mutex) != 0)
	    return GenerateSystemDescriptiveError(errno);

	return ERROR_NONE;
    }

    int Mutex::Lock(unsigned int msTimeout)
    {
	timespec time;
	int seconds = msTimeout / 1000;
	int nanoSeconds = (msTimeout - (seconds * 1000)) * 1000000;

	time.tv_sec = seconds;
	time.tv_nsec = nanoSeconds;

	if(mutex == NULL)
	    return ERROR_INVALID_PARAMETER;

	if(pthread_mutex_timedlock(mutex, &time) != 0)
	    return GenerateSystemDescriptiveError(errno);

	return ERROR_NONE;
    }

    int Mutex::TryLock()
    {
	if(mutex == NULL)
	    return ERROR_INVALID_PARAMETER;

	if(pthread_mutex_trylock(mutex) != 0)
	    return GenerateSystemDescriptiveError(errno);

	return ERROR_NONE;
    }

    int Mutex::Unlock()
    {
	if(mutex == NULL)
	    return ERROR_INVALID_PARAMETER;

	if(pthread_mutex_unlock(mutex) != 0)
	    return GenerateSystemDescriptiveError(errno);

	return ERROR_NONE;
    }

}

}
