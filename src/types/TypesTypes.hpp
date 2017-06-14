#ifndef __TYPESTYPES_HPP__
#define __TYPESTYPES_HPP__

namespace SystemToolkit
{

namespace Types
{

	enum ERROR_TYPES
	{
		ERROR_NONE,
		ERROR_OUT_OF_MEMORY,
		ERROR_INVALID_REQUEST,

		ERROR_SYSTEM_DESCRIPTIVE = 0x40000000
	};

	inline int GenerateSystemDescriptiveError(int number)
	{
		return (ERROR_SYSTEM_DESCRIPTIVE | number);
	}

}

}

#endif
