#ifndef __COREMUTEX_HPP__
#define __COREMUTEX_HPP__

#ifdef _WIN32
        #include "windows/Mutex.hpp"
#else
        #include "linux/Mutex.hpp"
#endif

#endif
