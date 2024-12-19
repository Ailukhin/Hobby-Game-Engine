#include "MathEngine.h"

namespace EngineCore
{
	// Add Proxy here

	Vec3Proxy::Vec3Proxy(float a, float b, float c)
		: x(a), y(b), z(c)
	{

	}

	Vec3Proxy::operator float() const
	{
		float n1 = (x * x) + (y * y) + (z * z);
		return Trig::sqrt(n1);
	}

	float Vec3Proxy::operator*(const Vec3Proxy& r) const
	{
		float n1 = (Vec3Proxy)*this;
		float n2 = (Vec3Proxy)r;

		return n1 * n2;
	}

	bool Vec3Proxy::operator>(const Vec3Proxy& r) const
	{
		float n1 = (x * x) + (y * y) + (z * z);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z);

		return n1 > n2;
	}

	bool Vec3Proxy::operator<(const Vec3Proxy& r) const
	{
		float n1 = (x * x) + (y * y) + (z * z);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z);

		return n1 < n2;
	}

	bool Vec3Proxy::operator==(const Vec3Proxy& r) const
	{
		float n1 = (x * x) + (y * y) + (z * z);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z);

		return n1 == n2;
	}

	bool Vec3Proxy::operator>=(const Vec3Proxy& r) const
	{
		float n1 = (x * x) + (y * y) + (z * z);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z);

		return n1 >= n2;
	}

	bool Vec3Proxy::operator<=(const Vec3Proxy& r) const
	{
		float n1 = (x * x) + (y * y) + (z * z);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z);

		return n1 <= n2;
	}

	bool Vec3Proxy::operator!=(const Vec3Proxy& r) const
	{
		float n1 = (x * x) + (y * y) + (z * z);
		float n2 = (r.x * r.x) + (r.y * r.y) + (r.z * r.z);

		return n1 != n2;
	}

}