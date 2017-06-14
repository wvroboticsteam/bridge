#ifndef __SYSTEMCALLS_HPP__
#define __SYSTEMCALLS_HPP__

#ifdef _WIN32

#include <windows.h>

#elif __linux__

#include <unistd.h>

#endif

namespace SystemToolkit
{

namespace Core
{

#ifdef _WIN32

#define MSSleep(x) Sleep(x)

#elif __linux__

#define MSSleep(x) usleep(x * 1000)

#endif


}

}

#endif
