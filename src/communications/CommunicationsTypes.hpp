#ifndef __COMMUNICATIONSTYPES_HPP__
#define __COMMUNICATIONSTYPES_HPP__

namespace SystemToolkit
{

namespace Communications
{

	enum SOCKET_DIRECTION
	{
		SOCKET_DIR_UNKNOWN,
		SOCKET_DIR_CLIENT,
		SOCKET_DIR_SERVER
	};

	enum SOCKET_TYPE
	{
		SOCKET_TYPE_UNKNOWN,
		SOCKET_TYPE_TCP,
		SOCKET_TYPE_UDP
	};

	enum ERROR_TYPES
	{
		ERROR_NONE,
		ERROR_INVALID_PARAMETER,
		ERROR_PARAMETER_LOCKED,
		ERROR_INCORRECT_LOOPBACK,
		ERROR_NO_SERVER,
		ERROR_LACK_RESOURCES,
		ERROR_NOT_READY,

		ERROR_SYSTEM_DESCRIPTIVE = 0x40000000
	};

	inline int GenerateSystemDescriptiveError(int number)
	{
		return (ERROR_SYSTEM_DESCRIPTIVE | number);
	}

	inline int ExtractSystemError(int inVal)
	{
		return (inVal & (!ERROR_SYSTEM_DESCRIPTIVE));
	}

}

}

#endif
