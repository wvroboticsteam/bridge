#include "StandardMath.hpp"
#include <math.h>

namespace SystemToolkit
{

namespace Math
{

	int Round(const float& value)
	{
		return (int)floor(value + 0.5);
	}

	int Round(const double& value)
	{
		return (int)floor(value + 0.5);
	}

}

}
