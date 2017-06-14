#ifndef __MATHTYPES_HPP__
#define __MATHTYPES_HPP__

namespace SystemToolkit
{

namespace Math
{

	enum ERROR_TYPES
	{
		ERROR_NONE,
		ERROR_INVALID_PARAMETER,
		ERROR_INVALID_REQUEST,

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
