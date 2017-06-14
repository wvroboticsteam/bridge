#ifndef __MUTEX_HPP__
#define __MUTEX_HPP__

#include <pthread.h>

namespace SystemToolkit
{

namespace Core
{
    class Mutex
    {
    public:
	Mutex(bool=true);
	~Mutex();

	int Initialize();
	int Deallocate();
	bool Ready();

	int Lock();
	int Lock(unsigned int);
	int TryLock();
	int Unlock();

    protected:
	pthread_mutex_t *mutex;
    };
}

}

#endif
