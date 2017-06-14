#ifndef __CORETHREAD_HPP__
#define __CORETHREAD_HPP__

#ifdef _WIN32
        #include "windows/Thread.hpp"
#else
        #include "linux/Thread.hpp"
        #include "linux/StaticThread.hpp"
#endif

#endif
