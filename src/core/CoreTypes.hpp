#ifndef __CORETYPES_HPP__
#define __CORETYPES_HPP__

namespace SystemToolkit
{

namespace Core
{

	enum ERROR_TYPES
	{
		ERROR_NONE,
		ERROR_RESOURCE_BUSY,
		ERROR_INVALID_PARAMETER,
		ERROR_ILLEGAL_REQUEST,

		ERROR_SYSTEM_SPECIFIC = 0x40000000
	};

	inline int GenerateSystemDescriptiveError(int errorNumber)
	{
		return (ERROR_SYSTEM_SPECIFIC | errorNumber);
	}
}

}

#endif
