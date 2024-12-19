#include <math.h>   
#include "Trig.h"

namespace EngineCore
{
	float Trig::cos(const float angle_radians)
	{
		return cosf(angle_radians);
	}

	float Trig::sin(const float angle_radians)
	{
		return sinf(angle_radians);
	}

	float Trig::tan(const float val)
	{
		return tanf(val);
	}

	float Trig::atan(const float val)
	{
		return atanf(val);
	}

	float Trig::atan2(const float y, const float x)
	{
		return atan2f(y, x);
	}

	float Trig::acos(const float val)
	{
		return acosf(val);
	}

	float Trig::asin(const float val)
	{
		return asinf(val);
	}

	void Trig::cossin(float& cos, float& sin, const float angle_radians)
	{
		cos = cosf(angle_radians);
		sin = sinf(angle_radians);
	}

	float Trig::sqrt(const float val)
	{
		return sqrtf(val);
	}

	float Trig::rsqrt(const float val)
	{
		// Fast inverse sqrt, good approximation
		// https://en.wikipedia.org/wiki/Fast_inverse_square_root

		/*const float threehalfs = 1.5F;
		float y = val;

		long i = *(long*)&y;

		i = 0x5f3759df - (i >> 1);
		y = *(float*)&i;

		y = y * (threehalfs - ((val * 0.5F) * y * y));*/

		// Slower way here
		return 1 / sqrtf(val);
	}
}