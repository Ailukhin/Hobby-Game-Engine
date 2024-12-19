#include "MathEngine.h"
#include <cstdlib>

namespace EngineCore
{
	Mat3::Mat3()
		: _v0(0.0f, 0.0f, 0.0f, 0.0f),
		_v1(0.0f, 0.0f, 0.0f, 0.0f),
		_v2(0.0f, 0.0f, 0.0f, 0.0f),
		_v3(0.0f, 0.0f, 0.0f, 0.0f)
	{

	}

	Mat3& Mat3::operator=(const Mat3& A)
	{
		_v0 = A._v0;
		_v1 = A._v1;
		_v2 = A._v2;

		return *this;
	}

	Mat3::Mat3(const Mat3& tM)
	{
		_v0 = tM._v0;
		_v1 = tM._v1;
		_v2 = tM._v2;
	}

	Mat3::~Mat3()
	{

	}

	Mat3::Mat3(const Vec3& tV0, const Vec3& tV1, const Vec3& tV2)
	{
		_v0 = Vec4(tV0, 1);
		_v1 = Vec4(tV1, 1);
		_v2 = Vec4(tV2, 1);
	}

	Mat3::Mat3(const Mat4& m)
	{
		_m0 = m._m0;
		_m1 = m._m1;
		_m2 = m._m2;
		_m3 = m._m3;
		_m4 = m._m4;
		_m5 = m._m5;
		_m6 = m._m6;
		_m7 = m._m7;
		_m8 = m._m8;
		_m9 = m._m9;
		_m10 = m._m10;
		_m11 = m._m11;
		_m12 = m._m12;
		_m13 = m._m13;
		_m14 = m._m14;
		_m15 = m._m15;
	}

	Mat3::Mat3(const Special type)
	{
		switch (type)
		{
		case Special::Zero:
			_m0 = 0.0f;
			_m1 = 0.0f;
			_m2 = 0.0f;
			_m3 = 0.0f;
			_m4 = 0.0f;
			_m5 = 0.0f;
			_m6 = 0.0f;
			_m7 = 0.0f;
			_m8 = 0.0f;
			_m9 = 0.0f;
			_m10 = 0.0f;
			_m11 = 0.0f;
			_m12 = 0.0f;
			_m13 = 0.0f;
			_m14 = 0.0f;
			_m15 = 0.0f;
			break;
		case Special::Identity:
			_m0 = 1.0f;
			_m1 = 0.0f;
			_m2 = 0.0f;
			_m3 = 0.0f;
			_m4 = 0.0f;
			_m5 = 1.0f;
			_m6 = 0.0f;
			_m7 = 0.0f;
			_m8 = 0.0f;
			_m9 = 0.0f;
			_m10 = 1.0f;
			_m11 = 0.0f;
			_m12 = 0.0f;
			_m13 = 0.0f;
			_m14 = 0.0f;
			_m15 = 0.0f;
			break;
		}
	}

	Vec3 Mat3::get(const Row3 type) const
	{
		Vec3 ret;

		switch (type)
		{
			case Row3::i0:
				ret = Vec3(_v0._vx, _v0._vy, _v0._vz);
				break;
			case Row3::i1:
				ret = Vec3(_v1._vx, _v1._vy, _v1._vz);
				break;
			case Row3::i2:
				ret = Vec3(_v2._vx, _v2._vy, _v2._vz);
				break;
		}

		return ret;
	}

	void Mat3::set(const Mat3& mIn)
	{
		_m0 = mIn._m0;
		_m1 = mIn._m1;
		_m2 = mIn._m2;
		_m3 = mIn._m3;
		_m4 = mIn._m4;
		_m5 = mIn._m5;
		_m6 = mIn._m6;
		_m7 = mIn._m7;
		_m8 = mIn._m8;
		_m9 = mIn._m9;
		_m10 = mIn._m10;
		_m11 = mIn._m11;
		_m12 = mIn._m12;
		_m13 = mIn._m13;
		_m14 = mIn._m14;
		_m15 = mIn._m15;
	}

	void Mat3::set(const Special type)
	{
		switch (type)
		{
			case Special::Zero:
				_m0 = 0.0f;
				_m1 = 0.0f;
				_m2 = 0.0f;
				_m3 = 0.0f;
				_m4 = 0.0f;
				_m5 = 0.0f;
				_m6 = 0.0f;
				_m7 = 0.0f;
				_m8 = 0.0f;
				_m9 = 0.0f;
				_m10 = 0.0f;
				_m11 = 0.0f;
				_m12 = 0.0f;
				_m13 = 0.0f;
				_m14 = 0.0f;
				_m15 = 0.0f;
				break;
			case Special::Identity:
				_m0 = 1.0f;
				_m1 = 0.0f;
				_m2 = 0.0f;
				_m3 = 0.0f;
				_m4 = 0.0f;
				_m5 = 1.0f;
				_m6 = 0.0f;
				_m7 = 0.0f;
				_m8 = 0.0f;
				_m9 = 0.0f;
				_m10 = 1.0f;
				_m11 = 0.0f;
				_m12 = 0.0f;
				_m13 = 0.0f;
				_m14 = 0.0f;
				_m15 = 0.0f;
				break;
		}
	}

	void Mat3::set(const Row3 type, const Vec3& V)
	{
		switch (type)
		{
			case Row3::i0:
				_v0 = Vec4(V, 1);
				break;
			case Row3::i1:
				_v1 = Vec4(V, 1);
				break;
			case Row3::i2:
				_v2 = Vec4(V, 1);
				break;
		}
	}

	void Mat3::set(const Vec3& V0, const Vec3& V1, const Vec3& V2)
	{
		_v0 = Vec4(V0, 1);
		_v1 = Vec4(V1, 1);
		_v2 = Vec4(V2, 1);
	}

	float &Mat3::operator[](const m0_enum)
	{
		return _m0;
	}

	float &Mat3::operator[](const m1_enum)
	{
		return _m1;
	}

	float &Mat3::operator[](const m2_enum)
	{
		return _m2;
	}

	float &Mat3::operator[](const m4_enum)
	{
		return _m4;
	}

	float &Mat3::operator[](const m5_enum)
	{
		return _m5;
	}

	float &Mat3::operator[](const m6_enum)
	{
		return _m6;
	}

	float &Mat3::operator[](const m8_enum)
	{
		return _m8;
	}

	float &Mat3::operator[](const m9_enum)
	{
		return _m9;
	}

	float &Mat3::operator[](const m10_enum)
	{
		return _m10;
	}

	float Mat3::operator[](const m0_enum) const
	{
		return _m0;
	}

	float Mat3::operator[](const m1_enum) const
	{
		return _m1;
	}

	float Mat3::operator[](const m2_enum) const
	{
		return _m2;
	}

	float Mat3::operator[](const m4_enum) const
	{
		return _m4;
	}

	float Mat3::operator[](const m5_enum) const
	{
		return _m5;
	}

	float Mat3::operator[](const m6_enum) const
	{
		return _m6;
	}

	float Mat3::operator[](const m8_enum) const
	{
		return _m8;
	}

	float Mat3::operator[](const m9_enum) const
	{
		return _m9;
	}

	float Mat3::operator[](const m10_enum) const
	{
		return _m10;
	}

	void Mat3::m0(const float v)
	{
		_m0 = v;
	}

	void Mat3::m1(const float v)
	{
		_m1 = v;
	}

	void Mat3::m2(const float v)
	{
		_m2 = v;
	}

	void Mat3::m4(const float v)
	{
		_m4 = v;
	}

	void Mat3::m5(const float v)
	{
		_m5 = v;
	}

	void Mat3::m6(const float v)
	{
		_m6 = v;
	}

	void Mat3::m8(const float v)
	{
		_m8 = v;
	}

	void Mat3::m9(const float v)
	{
		_m9 = v;
	}

	void Mat3::m10(const float v)
	{
		_m10 = v;
	}

	float Mat3::m0() const
	{
		return _m0;
	}

	float Mat3::m1() const
	{
		return _m1;
	}

	float Mat3::m2() const
	{
		return _m2;
	}

	float Mat3::m4() const
	{
		return _m4;
	}

	float Mat3::m5() const
	{
		return _m5;
	}

	float Mat3::m6() const
	{
		return _m6;
	}

	float Mat3::m8() const
	{
		return _m8;
	}

	float Mat3::m9() const
	{
		return _m9;
	}

	float Mat3::m10() const
	{
		return _m10;
	}

	float Mat3::det() const
	{
		float first = _m0 * (_m5 * _m10 - _m6 * _m9);
		float second = _m1 * (_m4 * _m10 - _m6 * _m8);
		float third = _m2 * (_m4 * _m9 - _m5 * _m8);

		return first - second + third;
	}

	Mat3& Mat3::T(void)
	{
		Vec3 x(_m0, _m4, _m8);
		Vec3 y(_m1, _m5, _m9);
		Vec3 z(_m2, _m6, _m10);

		_v0 = Vec4(x, 1);
		_v1 = Vec4(y, 1);
		_v2 = Vec4(z, 1);

		return *this;
	}

	Mat3 Mat3::getT(void) const
	{
		Vec3 x(_m0, _m4, _m8);
		Vec3 y(_m1, _m5, _m9);
		Vec3 z(_m2, _m6, _m10);

		return Mat3(x, y, z);
	}

	Mat3 Mat3::getInv(void) const
	{
		float detInv = 1 / this->det();

		Vec3 x(_m5 * _m10 - _m6 * _m9, _m2 * _m9 - _m1 * _m10, _m1 * _m6 - _m2 * _m5);
		Vec3 y(_m6 * _m8 - _m4 * _m10, _m0 * _m10 - _m2 * _m8, _m2 * _m4 - _m0 * _m6);
		Vec3 z(_m4 * _m9 - _m5 * _m8, _m1 * _m8 - _m0 * _m9, _m0 * _m5 - _m1 * _m4);

		return detInv * Mat3(x, y, z);
	}

	Mat3& Mat3::inv(void)
	{
		float detInv = 1 / this->det();

		Vec3 x(_m5 * _m10 - _m6 * _m9, _m2 * _m9 - _m1 * _m10, _m1 * _m6 - _m2 * _m5);
		Vec3 y(_m6 * _m8 - _m4 * _m10, _m0 * _m10 - _m2 * _m8, _m2 * _m4 - _m0 * _m6);
		Vec3 z(_m4 * _m9 - _m5 * _m8, _m1 * _m8 - _m0 * _m9, _m0 * _m5 - _m1 * _m4);

		_v0 = detInv * Vec4(x, 1);
		_v1 = detInv * Vec4(y, 1);
		_v2 = detInv * Vec4(z, 1);

		return *this;
	}

	bool Mat3::isEqual(const Mat3& A, const float epsilon) const
	{
		bool ret = true;

		if (fabsf(A._m0 - _m0) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m1 - _m1) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m2 - _m2) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m4 - _m4) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m5 - _m5) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m6 - _m6) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m8 - _m8) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m9 - _m9) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m10 - _m10) > epsilon)
		{
			ret = false;
		}

		return ret;
	}

	bool Mat3::isIdentity(const float epsilon) const
	{
		bool ret = true;

		// Check 0, 5, 10 first since those are 1.0f
		if (_m0 > (1.0f + epsilon) || _m0 < (- 1.0f - epsilon))
		{
			ret = false;
		}
		else if (_m5 > (1.0f + epsilon) || _m5 < (-1.0f - epsilon))
		{
			ret = false;
		}
		else if (_m10 > (1.0f + epsilon) || _m10 < (-1.0f - epsilon))
		{
			ret = false;
		}
		else if (_m1 > epsilon || _m1 < -epsilon)
		{
			ret = false;
		}
		else if (_m2 > epsilon || _m2 < -epsilon)
		{
			ret = false;
		}
		else if (_m4 > epsilon || _m4 < -epsilon)
		{
			ret = false;
		}
		else if (_m6 > epsilon || _m6 < -epsilon)
		{
			ret = false;
		}
		else if (_m8 > epsilon || _m8 < -epsilon)
		{
			ret = false;
		}
		else if (_m9 > epsilon || _m9 < -epsilon)
		{
			ret = false;
		}

		return ret;
	}

	Mat3 Mat3::operator+(void) const
	{
		return *this;
	}

	Mat3 Mat3::operator+(const Mat3& A) const
	{
		float m0 = _m0 + A._m0;
		float m1 = _m1 + A._m1;
		float m2 = _m2 + A._m2;

		float m4 = _m4 + A._m4;
		float m5 = _m5 + A._m5;
		float m6 = _m6 + A._m6;

		float m8 = _m8 + A._m8;
		float m9 = _m9 + A._m9;
		float m10 = _m10 + A._m10;

		Vec3 x(m0, m1, m2);
		Vec3 y(m4, m5, m6);
		Vec3 z(m8, m9, m10);

		return Mat3(x, y, z);
	}

	Mat3& Mat3::operator+=(const Mat3& A)
	{
		_m0 += A._m0;
		_m1 += A._m1;
		_m2 += A._m2;

		_m4 += A._m4;
		_m5 += A._m5;
		_m6 += A._m6;

		_m8 += A._m8;
		_m9 += A._m9;
		_m10 += A._m10;

		return *this;
	}

	Mat3 Mat3::operator-(void) const
	{
		Vec3 x(-_m0, -_m1, -_m2);
		Vec3 y(-_m4, -_m5, -_m6);
		Vec3 z(-_m8, -_m9, -_m10);

		return Mat3(x, y, z);
	}

	Mat3 Mat3::operator-(const Mat3& A) const
	{
		float m0 = _m0 - A._m0;
		float m1 = _m1 - A._m1;
		float m2 = _m2 - A._m2;

		float m4 = _m4 - A._m4;
		float m5 = _m5 - A._m5;
		float m6 = _m6 - A._m6;

		float m8 = _m8 - A._m8;
		float m9 = _m9 - A._m9;
		float m10 = _m10 - A._m10;

		Vec3 x(m0, m1, m2);
		Vec3 y(m4, m5, m6);
		Vec3 z(m8, m9, m10);

		return Mat3(x, y, z);
	}

	Mat3& Mat3::operator-=(const Mat3& A)
	{
		_m0 -= A._m0;
		_m1 -= A._m1;
		_m2 -= A._m2;

		_m4 -= A._m4;
		_m5 -= A._m5;
		_m6 -= A._m6;

		_m8 -= A._m8;
		_m9 -= A._m9;
		_m10 -= A._m10;

		return *this;
	}

	Mat3 Mat3::operator*(const float s) const
	{
		float m0 = _m0 * s;
		float m1 = _m1 * s;
		float m2 = _m2 * s;

		float m4 = _m4 * s;
		float m5 = _m5 * s;
		float m6 = _m6 * s;

		float m8 = _m8 * s;
		float m9 = _m9 * s;
		float m10 = _m10 * s;

		Vec3 x(m0, m1, m2);
		Vec3 y(m4, m5, m6);
		Vec3 z(m8, m9, m10);

		return Mat3(x, y, z);
	}

	Mat3& Mat3::operator*=(const float scale)
	{
		_m0 *= scale;
		_m1 *= scale;
		_m2 *= scale;

		_m4 *= scale;
		_m5 *= scale;
		_m6 *= scale;

		_m8 *= scale;
		_m9 *= scale;
		_m10 *= scale;

		return *this;
	}

	Mat3 operator*(const float scale, const Mat3& A)
	{
		float m0 = A._m0 * scale;
		float m1 = A._m1 * scale;
		float m2 = A._m2 * scale;

		float m4 = A._m4 * scale;
		float m5 = A._m5 * scale;
		float m6 = A._m6 * scale;

		float m8 = A._m8 * scale;
		float m9 = A._m9 * scale;
		float m10 = A._m10 * scale;

		Vec3 x(m0, m1, m2);
		Vec3 y(m4, m5, m6);
		Vec3 z(m8, m9, m10);

		return Mat3(x, y, z);
	}

	Mat3 Mat3::operator*(const Mat3& A) const
	{
		float m0 = _m0 * A._m0 + _m1 * A._m4 + _m2 * A._m8;
		float m1 = _m0 * A._m1 + _m1 * A._m5 + _m2 * A._m9;
		float m2 = _m0 * A._m2 + _m1 * A._m6 + _m2 * A._m10;

		float m4 = _m4 * A._m0 + _m5 * A._m4 + _m6 * A._m8;
		float m5 = _m4 * A._m1 + _m5 * A._m5 + _m6 * A._m9;
		float m6 = _m4 * A._m2 + _m5 * A._m6 + _m6 * A._m10;

		float m8 = _m8 * A._m0 + _m9 * A._m4 + _m10 * A._m8;
		float m9 = _m8 * A._m1 + _m9 * A._m5 + _m10 * A._m9;
		float m10 = _m8 * A._m2 + _m9 * A._m6 + _m10 * A._m10;

		Vec3 x(m0, m1, m2);
		Vec3 y(m4, m5, m6);
		Vec3 z(m8, m9, m10);

		return Mat3(x, y, z);
	}

	Mat3& Mat3::operator*=(const Mat3& A)
	{
		float m0 = _m0 * A._m0 + _m1 * A._m4 + _m2 * A._m8;
		float m1 = _m0 * A._m1 + _m1 * A._m5 + _m2 * A._m9;
		float m2 = _m0 * A._m2 + _m1 * A._m6 + _m2 * A._m10;

		float m4 = _m4 * A._m0 + _m5 * A._m4 + _m6 * A._m8;
		float m5 = _m4 * A._m1 + _m5 * A._m5 + _m6 * A._m9;
		float m6 = _m4 * A._m2 + _m5 * A._m6 + _m6 * A._m10;

		float m8 = _m8 * A._m0 + _m9 * A._m4 + _m10 * A._m8;
		float m9 = _m8 * A._m1 + _m9 * A._m5 + _m10 * A._m9;
		float m10 = _m8 * A._m2 + _m9 * A._m6 + _m10 * A._m10;

		_m0 = m0;
		_m1 = m1;
		_m2 = m2;

		_m4 = m4;
		_m5 = m5;
		_m6 = m6;

		_m8 = m8;
		_m9 = m9;
		_m10 = m10;

		return *this;
	}

}