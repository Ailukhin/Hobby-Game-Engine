#include "MathEngine.h"

namespace EngineCore
{
	Vec4Proxy::Vec4Proxy(float a, float b, float c, float d)
	{
		x = a;
		y = b;
		z = c;
		w = d;
	}

	Vec4Proxy::operator float() const
	{
		float n1 = (x * x) + (y * y) + (z * z) + (w * w);
		return Trig::sqrt(n1);
	}

	float Vec4Proxy::operator*(const Vec4Proxy& r) const
	{
		float n1 = (x * x) + (y * y) + (z * z) + (w * w);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z) + (r.w * r.w);

		return Trig::sqrt(n1) * Trig::sqrt(n2);
	}

	bool Vec4Proxy::operator>(const Vec4Proxy& r) const
	{
		bool ret = false;

		float n1 = (x * x) + (y * y) + (z * z) + (w * w);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z) + (r.w * r.w);

		if (n1 > n2)
		{
			ret = true;
		}

		return ret;
	}

	bool Vec4Proxy::operator<(const Vec4Proxy& r) const
	{
		bool ret = false;

		float n1 = (x * x) + (y * y) + (z * z) + (w * w);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z) + (r.w * r.w);

		if (n1 < n2)
		{
			ret = true;
		}

		return ret;
	}

	bool Vec4Proxy::operator==(const Vec4Proxy& r) const
	{
		bool ret = false;

		float n1 = (x * x) + (y * y) + (z * z) + (w * w);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z) + (r.w * r.w);

		if (n1 == n2)
		{
			ret = true;
		}

		return ret;
	}

	bool Vec4Proxy::operator>=(const Vec4Proxy& r) const
	{
		bool ret = false;

		float n1 = (x * x) + (y * y) + (z * z) + (w * w);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z) + (r.w * r.w);

		if (n1 >= n2)
		{
			ret = true;
		}

		return ret;
	}

	bool Vec4Proxy::operator<=(const Vec4Proxy& r) const
	{
		bool ret = false;

		float n1 = (x * x) + (y * y) + (z * z) + (w * w);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z) + (r.w * r.w);

		if (n1 <= n2)
		{
			ret = true;
		}

		return ret;
	}

	bool Vec4Proxy::operator!=(const Vec4Proxy& r) const
	{
		bool ret = false;

		float n1 = (x * x) + (y * y) + (z * z) + (w * w);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z) + (r.w * r.w);

		if (n1 != n2)
		{
			ret = true;
		}

		return ret;
	}

}