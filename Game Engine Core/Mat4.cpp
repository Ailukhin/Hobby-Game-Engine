#include "MathEngine.h"
#include <cstdlib>

using namespace EngineCore;

namespace EngineCore
{
	Mat4::Mat4()
		: _v0(0.0f, 0.0f, 0.0f, 0.0f),
		_v1(0.0f, 0.0f, 0.0f, 0.0f),
		_v2(0.0f, 0.0f, 0.0f, 0.0f),
		_v3(0.0f, 0.0f, 0.0f, 0.0f)
	{
		this->privSetGeneralHint();
	}

	Mat4& Mat4::operator=(const Mat4& A)
	{
		this->_v0 = A._v0;
		this->_v1 = A._v1;
		this->_v2 = A._v2;
		this->_v3 = A._v3;

		this->privSetGeneralHint();
		
		return *this;
	}

	Mat4::Mat4(const Mat4& tM)
	{
		this->_v0 = tM._v0;
		this->_v1 = tM._v1;
		this->_v2 = tM._v2;
		this->_v3 = tM._v3;

		this->privSetGeneralHint();
	}

	Mat4::~Mat4()
	{

	}

	Mat4& Mat4::operator=(const Quat& q)
	{
		this->_m0 = 1.0f - 2.0f * (q._qy * q._qy + q._qz * q._qz);
		this->_m1 = 2.0f * (q._qx * q._qy + q._qw * q._qz);
		this->_m2 = 2.0f * (q._qx * q._qz - q._qw * q._qy);
		this->_m3 = 0.0f;
		this->_m4 = 2.0f * (q._qx * q._qy - q._qw * q._qz);
		this->_m5 = 1.0f - 2.0f * (q._qx * q._qx + q._qz * q._qz);
		this->_m6 = 2.0f * (q._qy * q._qz + q._qw * q._qx);
		this->_m7 = 0.0f;
		this->_m8 = 2.0f * (q._qx * q._qz + q._qw * q._qy);
		this->_m9 = 2.0f * (q._qy * q._qz - q._qw * q._qx);
		this->_m10 = 1.0f - 2.0f * (q._qx * q._qx + q._qy * q._qy);
		this->_m11 = 0.0f;
		this->_m12 = 0.0f;
		this->_m13 = 0.0f;
		this->_m14 = 0.0f;
		this->_m15 = 1.0f;

		//this->privSetGeneralHint();
		this->privSetRotHint();

		return *this;
	}

	Mat4::Mat4(const Vec4& tV0, const Vec4& tV1, const Vec4& tV2, const Vec4& tV3)
		: _v0(tV0), _v1(tV1), _v2(tV2), _v3(tV3)
	{
		this->privSetGeneralHint();
	}

	Mat4::Mat4(const Special type)
	{
		switch (type)
		{
			case Special::Identity:
				this->_m0 = 1.0f;
				this->_m1 = 0.0f;
				this->_m2 = 0.0f;
				this->_m3 = 0.0f;
				this->_m4 = 0.0f;
				this->_m5 = 1.0f;
				this->_m6 = 0.0f;
				this->_m7 = 0.0f;
				this->_m8 = 0.0f;
				this->_m9 = 0.0f;
				this->_m10 = 1.0f;
				this->_m11 = 0.0f;
				this->_m12 = 0.0f;
				this->_m13 = 0.0f;
				this->_m14 = 0.0f;
				this->_m15 = 1.0f;
				break;
			case Special::Zero:
				this->_m0 = 0.0f;
				this->_m1 = 0.0f;
				this->_m2 = 0.0f;
				this->_m3 = 0.0f;
				this->_m4 = 0.0f;
				this->_m5 = 0.0f;
				this->_m6 = 0.0f;
				this->_m7 = 0.0f;
				this->_m8 = 0.0f;
				this->_m9 = 0.0f;
				this->_m10 = 0.0f;
				this->_m11 = 0.0f;
				this->_m12 = 0.0f;
				this->_m13 = 0.0f;
				this->_m14 = 0.0f;
				this->_m15 = 0.0f;
				break;
		}
	}

	Mat4::Mat4(const Quat& q)
	{
		this->_m0 = 1.0f - 2.0f * (q._qy * q._qy + q._qz * q._qz);
		this->_m1 = 2.0f * (q._qx * q._qy + q._qw * q._qz);
		this->_m2 = 2.0f * (q._qx * q._qz - q._qw * q._qy);
		this->_m3 = 0.0f;
		this->_m4 = 2.0f * (q._qx * q._qy - q._qw * q._qz);
		this->_m5 = 1.0f - 2.0f * (q._qx * q._qx + q._qz * q._qz);
		this->_m6 = 2.0f * (q._qy * q._qz + q._qw * q._qx);
		this->_m7 = 0.0f;
		this->_m8 = 2.0f * (q._qx * q._qz + q._qw * q._qy);
		this->_m9 = 2.0f * (q._qy * q._qz - q._qw * q._qx);
		this->_m10 = 1.0f - 2.0f * (q._qx * q._qx + q._qy * q._qy);
		this->_m11 = 0.0f;
		this->_m12 = 0.0f;
		this->_m13 = 0.0f;
		this->_m14 = 0.0f;
		this->_m15 = 1.0f;

		//this->privSetGeneralHint();
		this->privSetRotHint();
	}

	void Mat4::set(const Mat4& mIn)
	{
		this->_m0 = mIn._m0;
		this->_m1 = mIn._m1;
		this->_m2 = mIn._m2;
		this->_m3 = mIn._m3;
		this->_m4 = mIn._m4;
		this->_m5 = mIn._m5;
		this->_m6 = mIn._m6;
		this->_m7 = mIn._m7;
		this->_m8 = mIn._m8;
		this->_m9 = mIn._m9;
		this->_m10 = mIn._m10;
		this->_m11 = mIn._m11;
		this->_m12 = mIn._m12;
		this->_m13 = mIn._m13;
		this->_m14 = mIn._m14;
		this->_m15 = mIn._m15;

		this->privSetGeneralHint();
	}

	void Mat4::set(const Vec4& V0, const Vec4& V1, const Vec4& V2, const Vec4& V3)
	{
		this->_v0 = V0;
		this->_v1 = V1;
		this->_v2 = V2;
		this->_v3 = V3;

		this->privSetGeneralHint();
	}

	void Mat4::set(const Special type)
	{
		switch (type)
		{
			case Special::Identity:
				this->_m0 = 1.0f;
				this->_m1 = 0.0f;
				this->_m2 = 0.0f;
				this->_m3 = 0.0f;
				this->_m4 = 0.0f;
				this->_m5 = 1.0f;
				this->_m6 = 0.0f;
				this->_m7 = 0.0f;
				this->_m8 = 0.0f;
				this->_m9 = 0.0f;
				this->_m10 = 1.0f;
				this->_m11 = 0.0f;
				this->_m12 = 0.0f;
				this->_m13 = 0.0f;
				this->_m14 = 0.0f;
				this->_m15 = 1.0f;
				break;
			case Special::Zero:
				this->_m0 = 0.0f;
				this->_m1 = 0.0f;
				this->_m2 = 0.0f;
				this->_m3 = 0.0f;
				this->_m4 = 0.0f;
				this->_m5 = 0.0f;
				this->_m6 = 0.0f;
				this->_m7 = 0.0f;
				this->_m8 = 0.0f;
				this->_m9 = 0.0f;
				this->_m10 = 0.0f;
				this->_m11 = 0.0f;
				this->_m12 = 0.0f;
				this->_m13 = 0.0f;
				this->_m14 = 0.0f;
				this->_m15 = 0.0f;
				break;
		}
	}

	void Mat4::set(const Row4 type, const Vec4& V)
	{
		switch (type)
		{
			case Row4::i0:
				this->_v0 = V;
				break;
			case Row4::i1:
				this->_v1 = V;
				break;
			case Row4::i2:
				this->_v2 = V;
				break;
			case Row4::i3:
				this->_v3 = V;
				break;
		}

		this->privSetGeneralHint();
	}

	void Mat4::set(const Quat& q)
	{
		this->_m0 = 1.0f - 2.0f * (q._qy * q._qy + q._qz * q._qz);
		this->_m1 = 2.0f * (q._qx * q._qy + q._qw * q._qz);
		this->_m2 = 2.0f * (q._qx * q._qz - q._qw * q._qy);
		this->_m3 = 0.0f;
		this->_m4 = 2.0f * (q._qx * q._qy - q._qw * q._qz);
		this->_m5 = 1.0f - 2.0f * (q._qx * q._qx + q._qz * q._qz);
		this->_m6 = 2.0f * (q._qy * q._qz + q._qw * q._qx);
		this->_m7 = 0.0f;
		this->_m8 = 2.0f * (q._qx * q._qz + q._qw * q._qy);
		this->_m9 = 2.0f * (q._qy * q._qz - q._qw * q._qx);
		this->_m10 = 1.0f - 2.0f * (q._qx * q._qx + q._qy * q._qy);
		this->_m11 = 0.0f;
		this->_m12 = 0.0f;
		this->_m13 = 0.0f;
		this->_m14 = 0.0f;
		this->_m15 = 1.0f;

		//this->privSetGeneralHint();
		this->privSetRotHint();
	}

	Vec4 Mat4::get(const Row4 type) const
	{
		Vec4 ret;

		switch (type)
		{
			case Row4::i0:
				ret = _v0;
				break;
			case Row4::i1:
				ret = _v1;
				break;
			case Row4::i2:
				ret = _v2;
				break;
			case Row4::i3:
				ret = _v3;
				break;
		}

		return ret;
	}

	float Mat4::operator[](const m0_enum) const
	{
		return this->_m0;
	}

	float Mat4::operator[](const m1_enum) const
	{
		return this->_m1;
	}

	float Mat4::operator[](const m2_enum) const
	{
		return this->_m2;
	}

	float Mat4::operator[](const m3_enum) const
	{
		return this->_m3;
	}

	float Mat4::operator[](const m4_enum) const
	{
		return this->_m4;
	}

	float Mat4::operator[](const m5_enum) const
	{
		return this->_m5;
	}

	float Mat4::operator[](const m6_enum) const
	{
		return this->_m6;
	}

	float Mat4::operator[](const m7_enum) const
	{
		return this->_m7;
	}

	float Mat4::operator[](const m8_enum) const
	{
		return this->_m8;
	}

	float Mat4::operator[](const m9_enum) const
	{
		return this->_m9;
	}

	float Mat4::operator[](const m10_enum) const
	{
		return this->_m10;
	}

	float Mat4::operator[](const m11_enum) const
	{
		return this->_m11;
	}

	float Mat4::operator[](const m12_enum) const
	{
		return this->_m12;
	}

	float Mat4::operator[](const m13_enum) const
	{
		return this->_m13;
	}

	float Mat4::operator[](const m14_enum) const
	{
		return this->_m14;
	}

	float Mat4::operator[](const m15_enum) const
	{
		return this->_m15;
	}

	Mat4Proxy Mat4::operator[] (const enum m0_enum)
	{
		return Mat4Proxy(*this, this->_m0);
	}

	Mat4Proxy Mat4::operator[] (const enum m1_enum)
	{
		return Mat4Proxy(*this, this->_m1);
	}

	Mat4Proxy Mat4::operator[] (const enum m2_enum)
	{
		return Mat4Proxy(*this, this->_m2);
	}

	Mat4Proxy Mat4::operator[] (const enum m3_enum)
	{
		return Mat4Proxy(*this, this->_m3);
	}

	Mat4Proxy Mat4::operator[] (const enum m4_enum)
	{
		return Mat4Proxy(*this, this->_m4);
	}

	Mat4Proxy Mat4::operator[] (const enum m5_enum)
	{
		return Mat4Proxy(*this, this->_m5);
	}

	Mat4Proxy Mat4::operator[] (const enum m6_enum)
	{
		return Mat4Proxy(*this, this->_m6);
	}

	Mat4Proxy Mat4::operator[] (const enum m7_enum)
	{
		return Mat4Proxy(*this, this->_m7);
	}

	Mat4Proxy Mat4::operator[] (const enum m8_enum)
	{
		return Mat4Proxy(*this, this->_m8);
	}

	Mat4Proxy Mat4::operator[] (const enum m9_enum)
	{
		return Mat4Proxy(*this, this->_m9);
	}

	Mat4Proxy Mat4::operator[] (const enum m10_enum)
	{
		return Mat4Proxy(*this, this->_m10);
	}

	Mat4Proxy Mat4::operator[] (const enum m11_enum)
	{
		return Mat4Proxy(*this, this->_m11);
	}

	Mat4Proxy Mat4::operator[] (const enum m12_enum)
	{
		return Mat4Proxy(*this, this->_m12);
	}

	Mat4Proxy Mat4::operator[] (const enum m13_enum)
	{
		return Mat4Proxy(*this, this->_m13);
	}

	Mat4Proxy Mat4::operator[] (const enum m14_enum)
	{
		return Mat4Proxy(*this, this->_m14);
	}

	Mat4Proxy Mat4::operator[] (const enum m15_enum)
	{
		return Mat4Proxy(*this, this->_m15);
	}

	void Mat4::m0(const float v)
	{
		this->privSetGeneralHint();
		this->_m0 = v;
	}

	void Mat4::m1(const float v)
	{
		this->privSetGeneralHint();
		this->_m1 = v;
	}

	void Mat4::m2(const float v)
	{
		this->privSetGeneralHint();
		this->_m2 = v;
	}

	void Mat4::m3(const float v)
	{
		this->privSetGeneralHint();
		this->_m3 = v;
	}

	void Mat4::m4(const float v)
	{
		this->privSetGeneralHint();
		this->_m4 = v;
	}

	void Mat4::m5(const float v)
	{
		this->privSetGeneralHint();
		this->_m5 = v;
	}

	void Mat4::m6(const float v)
	{
		this->privSetGeneralHint();
		this->_m6 = v;
	}

	void Mat4::m7(const float v)
	{
		this->privSetGeneralHint();
		this->_m7 = v;
	}

	void Mat4::m8(const float v)
	{
		this->privSetGeneralHint();
		this->_m8 = v;
	}

	void Mat4::m9(const float v)
	{
		this->privSetGeneralHint();
		this->_m9 = v;
	}

	void Mat4::m10(const float v)
	{
		this->privSetGeneralHint();
		this->_m10 = v;
	}

	void Mat4::m11(const float v)
	{
		this->privSetGeneralHint();
		this->_m11 = v;
	}

	void Mat4::m12(const float v)
	{
		this->privSetGeneralHint();
		this->_m12 = v;
	}

	void Mat4::m13(const float v)
	{
		this->privSetGeneralHint();
		this->_m13 = v;
	}

	void Mat4::m14(const float v)
	{
		this->privSetGeneralHint();
		this->_m14 = v;
	}

	void Mat4::m15(const float v)
	{
		this->privSetGeneralHint();
		this->_m15 = v;
	}

	float Mat4::m0() const
	{
		return this->_m0;
	}

	float Mat4::m1() const
	{
		return this->_m1;
	}

	float Mat4::m2() const
	{
		return this->_m2;
	}

	float Mat4::m3() const
	{
		return this->_m3;
	}

	float Mat4::m4() const
	{
		return this->_m4;
	}

	float Mat4::m5() const
	{
		return this->_m5;
	}

	float Mat4::m6() const
	{
		return this->_m6;
	}

	float Mat4::m7() const
	{
		return this->_m7;
	}

	float Mat4::m8() const
	{
		return this->_m8;
	}

	float Mat4::m9() const
	{
		return this->_m9;
	}

	float Mat4::m10() const
	{
		return this->_m10;
	}

	float Mat4::m11() const
	{
		return this->_m11;
	}

	float Mat4::m12() const
	{
		return this->_m12;
	}

	float Mat4::m13() const
	{
		return this->_m13;
	}

	float Mat4::m14() const
	{
		return this->_m14;
	}

	float Mat4::m15() const
	{
		return this->_m15;
	}

	float Mat4::det() const
	{
		float A, B, C, D, E, F;

		/*     | m0  m1  m2  m3  | */
		/* m = | m4  m5  m6  m7  | */
		/*     | m8  m9  m10 m11 | */
		/*     | m12 m13 m14 m15 | */

		/* det(m) =  m0*[m5(m10*m15-m14*m11) -m6*(m9*m15-m13*m11) +m7*(m9*m14-m13*m10)] */
		/*          -m1*[m4(m10*m15-m14*m11) -m6*(m8*m15-m12*m11) +m7*(m8*m14-m12*m10)] */
		/*           m2*[m4(m9*m15-m13*m11)  -m5*(m8*m15-m12*m11) +m7*(m8*m13-m12*m9) ] */
		/*          -m3*[m4(m9*m14-m13*m10)  -m5*(m8*m14-m12*m10) +m6*(m8*m13-m12*m9) ] */

		/* let: A = m10*m15 - m14*m11 */
		/*      B = m8*m15  - m12*m11 */
		/*      C = m8*m13  - m12*m9  */
		/*      D = m8*m14  - m12*m10 */
		/*      E = m9*m15  - m13*m11 */
		/*      F = m9*m14  - m13*m10 */

		/* det(m) =  m0*[ m5*A - m6*E + m7*F ] */
		/*          -m1*[ m4*A - m6*B + m7*D ] */
		/*           m2*[ m4*E - m5*B + m7*C ] */
		/*          -m3*[ m4*F - m5*D + m6*C ] */

		A = (_m10 * _m15) - (_m14 * _m11);
		B = (_m8 * _m15) - (_m12 * _m11);
		C = (_m8 * _m13) - (_m12 * _m9);
		D = (_m8 * _m14) - (_m12 * _m10);
		E = (_m9 * _m15) - (_m13 * _m11);
		F = (_m9 * _m14) - (_m13 * _m10);

		return (_m0 * ((_m5 * A) - (_m6 * E) + (_m7 * F))
			- _m1 * ((_m4 * A) - (_m6 * B) + (_m7 * D))
			+ _m2 * ((_m4 * E) - (_m5 * B) + (_m7 * C))
			- _m3 * ((_m4 * F) - (_m5 * D) + (_m6 * C)));
	}

	Mat4 Mat4::getInv(void) const
	{
		Mat4 Inverse(*this);
		float invDet;

		Hint h = this->privGetHint();

		switch (h)
		{
		case Hint::Generalize:
			// Full 4x4 matrix inverse
			invDet = 1.0f / this->det();

			Inverse = this->privGetAdj();
			Inverse *= invDet;

			break;
		case Hint::Rot:
	
			// Manual transpose
			Inverse._m0 = this->_m0;
			Inverse._m1 = this->_m4;
			Inverse._m2 = this->_m8;

			Inverse._m4 = this->_m1;
			Inverse._m5 = this->_m5;
			Inverse._m6 = this->_m9;
	
			Inverse._m8 = this->_m2;
			Inverse._m9 = this->_m6;
			Inverse._m10 = this->_m10;

			break;
		case Hint::Trans:

			Inverse._m0 = this->_m0;
			Inverse._m5 = this->_m5;
			Inverse._m10 = this->_m10;

			Inverse._m12 = -this->_m12;
			Inverse._m13 = -this->_m13;
			Inverse._m14 = -this->_m14;

			Inverse._m15 = this->_m15;

			break;
		case Hint::RotTrans:

			// Manual transpose
			Inverse._m0 = this->_m0;
			Inverse._m1 = this->_m4;
			Inverse._m2 = this->_m8;

			Inverse._m4 = this->_m1;
			Inverse._m5 = this->_m5;
			Inverse._m6 = this->_m9;

			Inverse._m8 = this->_m2;
			Inverse._m9 = this->_m6;
			Inverse._m10 = this->_m10;

			// Translations (Translation Vec3 * Rot 3x3)
			Inverse._m12 = -this->_m12 * Inverse._m0 + -this->_m13 * Inverse._m4 + -this->_m14 * Inverse._m8;
			Inverse._m13 = -this->_m12 * Inverse._m1 + -this->_m13 * Inverse._m5 + -this->_m14 * Inverse._m9;
			Inverse._m14 = -this->_m12 * Inverse._m2 + -this->_m13 * Inverse._m6 + -this->_m14 * Inverse._m10;

			break;
		case Hint::Scale:

			Inverse._m0 = 1 / this->_m0;
			Inverse._m5 = 1 / this->_m5;
			Inverse._m10 = 1 / this->_m10;

			Inverse._m15 = this->_m15;

			break;
		case Hint::RotScale:
			
			// Rot and scale - manual 3x3 inverse
			invDet = 1.0f / ((_m0 * (_m5 * _m10 - _m6 * _m9)) - (_m1 * (_m4 * _m10 - _m6 * _m8)) + (_m2 * (_m4 * _m9 - _m5 * _m8)));

			Inverse._m0 = invDet * (_m5 * _m10 - _m6 * _m9);
			Inverse._m1 = invDet * (_m2 * _m9 - _m1 * _m10);
			Inverse._m2 = invDet * (_m1 * _m6 - _m2 * _m5);

			Inverse._m4 = invDet * (_m6 * _m8 - _m4 * _m10);
			Inverse._m5 = invDet * (_m0 * _m10 - _m2 * _m8);
			Inverse._m6 = invDet * (_m2 * _m4 - _m0 * _m6);

			Inverse._m8 = invDet * (_m4 * _m9 - _m5 * _m8);
			Inverse._m9 = invDet * (_m1 * _m8 - _m0 * _m9);
			Inverse._m10 = invDet * (_m0 * _m5 - _m1 * _m4);

			break;
		case Hint::TransScale:
			// Scale
			Inverse._m0 = 1 / this->_m0;
			Inverse._m5 = 1 / this->_m5;
			Inverse._m10 = 1 / this->_m10;

			// Translations (Translation Vec3 * 3x3 SCALE ONLY - so just operate on m0, m5, and m10)
			Inverse._m12 = -this->_m12 * Inverse._m0;
			Inverse._m13 = -this->_m13 * Inverse._m5;
			Inverse._m14 = -this->_m14 * Inverse._m10;

			break;
		case Hint::Affine:

			// Rot and scale - manual 3x3 inverse
			invDet = 1.0f / ((_m0 * (_m5 * _m10 - _m6 * _m9)) - (_m1 * (_m4 * _m10 - _m6 * _m8)) + (_m2 * (_m4 * _m9 - _m5 * _m8)));

			Inverse._m0 = invDet * (_m5 * _m10 - _m6 * _m9);
			Inverse._m1 = invDet * (_m2 * _m9 - _m1 * _m10);
			Inverse._m2 = invDet * (_m1 * _m6 - _m2 * _m5);

			Inverse._m4 = invDet * (_m6 * _m8 - _m4 * _m10);
			Inverse._m5 = invDet * (_m0 * _m10 - _m2 * _m8);
			Inverse._m6 = invDet * (_m2 * _m4 - _m0 * _m6);

			Inverse._m8 = invDet * (_m4 * _m9 - _m5 * _m8);
			Inverse._m9 = invDet * (_m1 * _m8 - _m0 * _m9);
			Inverse._m10 = invDet * (_m0 * _m5 - _m1 * _m4);

			// Translations (Translation Vec3 * Rot 3x3)
			Inverse._m12 = -this->_m12 * Inverse._m0 + -this->_m13 * Inverse._m4 + -this->_m14 * Inverse._m8;
			Inverse._m13 = -this->_m12 * Inverse._m1 + -this->_m13 * Inverse._m5 + -this->_m14 * Inverse._m9;
			Inverse._m14 = -this->_m12 * Inverse._m2 + -this->_m13 * Inverse._m6 + -this->_m14 * Inverse._m10;

			break;
		}

		return Inverse;
	}

	Mat4 Mat4::privGetAdj(void) const
	{
		float A, B, C, D, E, F, G, H, I;
		Mat4 tmp;

		A = (_m10 * _m15) - (_m11 * _m14);
		B = (_m7 * _m14) - (_m6 * _m15);
		C = (_m6 * _m11) - (_m7 * _m10);
		D = (_m3 * _m14) - (_m2 * _m15);
		E = (_m2 * _m11) - (_m3 * _m10);
		F = (_m2 * _m7) - (_m3 * _m6);
		G = _m5;
		H = _m13;

		/* 0		a = {5,  9, 13,  6, 10, 14,  7, 11, 15} */
		tmp._m0 = (G * A) + (_m9 * B) + (H * C);

		/* 2		a = {1,  5, 13,  2,  6, 14,  3,  7, 15} */
		tmp._m2 = -(_m1 * B) + (G * D) + (H * F);

		G = _m0;
		H = _m8;
		/* 5		a = {0,  8, 12,  2, 10, 14,  3, 11, 15} */
		tmp._m5 = (G * A) + (H * D) + (_m12 * E);

		/* 7		a = {0,  4,  8,  2,  6, 10,  3,  7, 11} */
		tmp._m7 = (G * C) + (H * F) - (_m4 * E);

		A = (_m9 * _m15) - (_m11 * _m13);
		B = (_m3 * _m13) - (_m1 * _m15);
		C = (_m1 * _m11) - (_m3 * _m9);
		D = (_m7 * _m13) - (_m5 * _m15);
		E = (_m5 * _m11) - (_m7 * _m9);
		F = (_m1 * _m7) - (_m3 * _m5);
		G = _m2;
		H = _m10;
		/* 1		a = {2, 10, 14,  1,  9, 13,  3, 11, 15} */
		tmp._m1 = (G * A) + (H * B) + (_m14 * C);

		/* 3		a = {2,  6, 10,  1,  5,  9,  3,  7, 11} */
		tmp._m3 = (G * E) + (H * F) - (_m6 * C);

		G = _m4;
		H = _m12;
		/* 8		a = {4,  8, 12,  5,  9, 13,  7, 11, 15} */
		tmp._m8 = (G * A) + (_m8 * D) + (H * E);

		/* A		a = {0,  4, 12,  1,  5, 13,  3,  7, 15} */
		tmp._m10 = -(_m0 * D) + (G * B) + (H * F);

		A = (_m8 * _m15) - (_m11 * _m12);
		B = (_m4 * _m11) - (_m7 * _m8);
		C = (_m3 * _m12) - (_m0 * _m15);
		D = (_m0 * _m7) - (_m3 * _m4);
		E = (_m7 * _m12) - (_m4 * _m15);
		F = (_m0 * _m11) - (_m3 * _m8);
		G = _m6;
		H = _m14;
		/* 4		a = {6, 10, 14,  4,  8, 12,  7, 11, 15} */
		tmp._m4 = (G * A) + (_m10 * E) + (H * B);

		/* 6		a = {2,  6, 14,  0,  4, 12,  3,  7, 15} */
		tmp._m6 = -(_m2 * E) + (G * C) + (H * D);

		G = _m1;
		H = _m9;
		/* 9		a = {1,  9, 13,  0,  8, 12,  3, 11, 15} */
		tmp._m9 = (G * A) + (H * C) + (_m13 * F);

		/* B		a = {1,  5,  9,  0,  4,  8,  3,  7, 11} */
		tmp._m11 = (G * B) + (H * D) - (_m5 * F);

		A = _m4;
		B = _m6;
		C = _m12;
		D = _m14;
		E = (B * C) - (A * D);
		F = _m8;
		G = _m10;
		H = _m5;
		I = _m13;
		/* C		a = {5,  9, 13,  4,  8, 12,  6, 10, 14} */
		tmp._m12 = (H * ((F * D) - (G * C))) + (_m9 * E) + (I * ((A * G) - (B * F)));

		F = _m2;
		G = _m0;
		/* E		a = {1,  5, 13,  0,  4, 12,  2,  6, 14} */
		tmp._m14 = -(_m1 * E) + (H * ((F * C) - (G * D))) + (I * ((G * B) - (F * A)));

		A = _m1;
		B = _m2;
		C = _m9;
		D = _m10;
		E = (A * D) - (B * C);
		F = _m13;
		G = _m14;
		H = _m0;
		I = _m8;
		/* D		a = {0,  8, 12,  1,  9, 13,  2, 10, 14} */
		tmp._m13 = (H * ((C * G) - (D * F))) + (I * ((B * F) - (A * G))) + (_m12 * E);

		F = _m5;
		G = _m6;
		/* F		a = {0,  4,  8,  1,  5,  9,  2,  6, 10}} */
		tmp._m15 = -(_m4 * E) + (H * ((F * D) - (G * C))) + (I * ((A * G) - (B * F)));

		return tmp;
	}

	Mat4& Mat4::inv(void)
	{
		Mat4 Inverse;
		float invDet;

		Hint h = this->privGetHint();

		switch (h)
		{
		case Hint::Generalize:
			// Full 4x4 matrix inverse
			invDet = 1.0f / this->det();

			Inverse = this->privGetAdj();
			Inverse *= invDet;

			break;
		case Hint::Rot:

			// Manual transpose
			Inverse._m0 = this->_m0;
			Inverse._m1 = this->_m4;
			Inverse._m2 = this->_m8;

			Inverse._m4 = this->_m1;
			Inverse._m5 = this->_m5;
			Inverse._m6 = this->_m9;

			Inverse._m8 = this->_m2;
			Inverse._m9 = this->_m6;
			Inverse._m10 = this->_m10;

			break;
		case Hint::Trans:

			Inverse._m0 = this->_m0;
			Inverse._m5 = this->_m5;
			Inverse._m10 = this->_m10;

			Inverse._m12 = -this->_m12;
			Inverse._m13 = -this->_m13;
			Inverse._m14 = -this->_m14;

			Inverse._m15 = this->_m15;

			break;
		case Hint::RotTrans:

			// Manual transpose
			Inverse._m0 = this->_m0;
			Inverse._m1 = this->_m4;
			Inverse._m2 = this->_m8;

			Inverse._m4 = this->_m1;
			Inverse._m5 = this->_m5;
			Inverse._m6 = this->_m9;

			Inverse._m8 = this->_m2;
			Inverse._m9 = this->_m6;
			Inverse._m10 = this->_m10;

			// Translations (Translation Vec3 * Rot 3x3)
			Inverse._m12 = -this->_m12 * Inverse._m0 + -this->_m13 * Inverse._m4 + -this->_m14 * Inverse._m8;
			Inverse._m13 = -this->_m12 * Inverse._m1 + -this->_m13 * Inverse._m5 + -this->_m14 * Inverse._m9;
			Inverse._m14 = -this->_m12 * Inverse._m2 + -this->_m13 * Inverse._m6 + -this->_m14 * Inverse._m10;

			break;
		case Hint::Scale:

			Inverse._m0 = 1 / this->_m0;
			Inverse._m5 = 1 / this->_m5;
			Inverse._m10 = 1 / this->_m10;

			Inverse._m15 = this->_m15;

			break;
		case Hint::RotScale:

			// Rot and scale - manual 3x3 inverse
			invDet = 1.0f / ((_m0 * (_m5 * _m10 - _m6 * _m9)) - (_m1 * (_m4 * _m10 - _m6 * _m8)) + (_m2 * (_m4 * _m9 - _m5 * _m8)));

			Inverse._m0 = invDet * (_m5 * _m10 - _m6 * _m9);
			Inverse._m1 = invDet * (_m2 * _m9 - _m1 * _m10);
			Inverse._m2 = invDet * (_m1 * _m6 - _m2 * _m5);

			Inverse._m4 = invDet * (_m6 * _m8 - _m4 * _m10);
			Inverse._m5 = invDet * (_m0 * _m10 - _m2 * _m8);
			Inverse._m6 = invDet * (_m2 * _m4 - _m0 * _m6);

			Inverse._m8 = invDet * (_m4 * _m9 - _m5 * _m8);
			Inverse._m9 = invDet * (_m1 * _m8 - _m0 * _m9);
			Inverse._m10 = invDet * (_m0 * _m5 - _m1 * _m4);

			break;
		case Hint::TransScale:
			// Scale
			Inverse._m0 = 1 / this->_m0;
			Inverse._m5 = 1 / this->_m5;
			Inverse._m10 = 1 / this->_m10;

			// Translations (Translation Vec3 * 3x3 SCALE ONLY - so just operate on m0, m5, and m10)
			Inverse._m12 = -this->_m12 * Inverse._m0;
			Inverse._m13 = -this->_m13 * Inverse._m5;
			Inverse._m14 = -this->_m14 * Inverse._m10;

			break;
		case Hint::Affine:

			// Rot and scale - manual 3x3 inverse
			invDet = 1.0f / ((_m0 * (_m5 * _m10 - _m6 * _m9)) - (_m1 * (_m4 * _m10 - _m6 * _m8)) + (_m2 * (_m4 * _m9 - _m5 * _m8)));

			Inverse._m0 = invDet * (_m5 * _m10 - _m6 * _m9);
			Inverse._m1 = invDet * (_m2 * _m9 - _m1 * _m10);
			Inverse._m2 = invDet * (_m1 * _m6 - _m2 * _m5);

			Inverse._m4 = invDet * (_m6 * _m8 - _m4 * _m10);
			Inverse._m5 = invDet * (_m0 * _m10 - _m2 * _m8);
			Inverse._m6 = invDet * (_m2 * _m4 - _m0 * _m6);

			Inverse._m8 = invDet * (_m4 * _m9 - _m5 * _m8);
			Inverse._m9 = invDet * (_m1 * _m8 - _m0 * _m9);
			Inverse._m10 = invDet * (_m0 * _m5 - _m1 * _m4);

			// Translations (Translation Vec3 * Rot 3x3)
			Inverse._m12 = -this->_m12 * Inverse._m0 + -this->_m13 * Inverse._m4 + -this->_m14 * Inverse._m8;
			Inverse._m13 = -this->_m12 * Inverse._m1 + -this->_m13 * Inverse._m5 + -this->_m14 * Inverse._m9;
			Inverse._m14 = -this->_m12 * Inverse._m2 + -this->_m13 * Inverse._m6 + -this->_m14 * Inverse._m10;

			break;
		}

		*this = Inverse;

		return *this;
	}

	Mat4& Mat4::T(void)
	{
		Vec4 x(_m0, _m4, _m8, _m12);
		Vec4 y(_m1, _m5, _m9, _m13);
		Vec4 z(_m2, _m6, _m10, _m14);
		Vec4 w(_m3, _m7, _m11, _m15);

		_v0 = x;
		_v1 = y;
		_v2 = z;
		_v3 = w;

		return *this;
	}

	Mat4 Mat4::getT(void) const
	{
		Vec4 x(_m0, _m4, _m8, _m12);
		Vec4 y(_m1, _m5, _m9, _m13);
		Vec4 z(_m2, _m6, _m10, _m14);
		Vec4 w(_m3, _m7, _m11, _m15);

		return Mat4(x, y, z, w);
	}

	bool Mat4::isEqual(const Mat4& A, const float epsilon) const
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
		else if (fabsf(A._m3 - _m3) > epsilon)
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
		else if (fabsf(A._m7 - _m7) > epsilon)
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
		else if (fabsf(A._m11 - _m11) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m12 - _m12) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m13 - _m13) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m14 - _m14) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(A._m15 - _m15) > epsilon)
		{
			ret = false;
		}

		return ret;
	}

	bool Mat4::isIdentity(const float epsilon) const
	{
		bool ret = true;

		// Check 0, 5, 10, 15 first since those are 1.0f
		if (_m0 > (1.0f + epsilon) || _m0 < (-1.0f - epsilon))
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
		else if (_m15 > (1.0f + epsilon) || _m15 < (-1.0f - epsilon))
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
		else if (_m3 > epsilon || _m3 < -epsilon)
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
		else if (_m7 > epsilon || _m7 < -epsilon)
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
		else if (_m11 > epsilon || _m11 < -epsilon)
		{
			ret = false;
		}
		else if (_m12 > epsilon || _m12 < -epsilon)
		{
			ret = false;
		}
		else if (_m13 > epsilon || _m13 < -epsilon)
		{
			ret = false;
		}
		else if (_m14 > epsilon || _m14 < -epsilon)
		{
			ret = false;
		}

		return ret;
	}

	/*const bool Mat4::isRotation(const float epsilon) const
	{




		return false;
	}*/

	Mat4 Mat4::operator+(void) const
	{
		return *this;
	}

	Mat4 Mat4::operator+(const Mat4& A) const
	{
		float m0 = _m0 + A._m0;
		float m1 = _m1 + A._m1;
		float m2 = _m2 + A._m2;
		float m3 = _m3 + A._m3;

		float m4 = _m4 + A._m4;
		float m5 = _m5 + A._m5;
		float m6 = _m6 + A._m6;
		float m7 = _m7 + A._m7;

		float m8 = _m8 + A._m8;
		float m9 = _m9 + A._m9;
		float m10 = _m10 + A._m10;
		float m11 = _m11 + A._m11;

		float m12 = _m12 + A._m12;
		float m13 = _m13 + A._m13;
		float m14 = _m14 + A._m14;
		float m15 = _m15 + A._m15;

		Vec4 x(m0, m1, m2, m3);
		Vec4 y(m4, m5, m6, m7);
		Vec4 z(m8, m9, m10, m11);
		Vec4 w(m12, m13, m14, m15);

		return Mat4(x, y, z, w);
	}

	Mat4& Mat4::operator+=(const Mat4& A)
	{
		_m0 += A._m0;
		_m1 += A._m1;
		_m2 += A._m2;
		_m3 += A._m3;

		_m4 += A._m4;
		_m5 += A._m5;
		_m6 += A._m6;
		_m7 += A._m7;

		_m8 += A._m8;
		_m9 += A._m9;
		_m10 += A._m10;
		_m11 += A._m11;

		_m12 += A._m12;
		_m13 += A._m13;
		_m14 += A._m14;
		_m15 += A._m15;

		return *this;
	}

	Mat4 Mat4::operator-(void) const
	{
		Vec4 x(-_m0, -_m1, -_m2, -_m3);
		Vec4 y(-_m4, -_m5, -_m6, -_m7);
		Vec4 z(-_m8, -_m9, -_m10, -_m11);
		Vec4 w(-_m12, -_m13, -_m14, -_m15);

		return Mat4(x, y, z, w);
	}

	Mat4 Mat4::operator-(const Mat4& A) const
	{
		float m0 = _m0 - A._m0;
		float m1 = _m1 - A._m1;
		float m2 = _m2 - A._m2;
		float m3 = _m3 - A._m3;

		float m4 = _m4 - A._m4;
		float m5 = _m5 - A._m5;
		float m6 = _m6 - A._m6;
		float m7 = _m7 - A._m7;

		float m8 = _m8 - A._m8;
		float m9 = _m9 - A._m9;
		float m10 = _m10 - A._m10;
		float m11 = _m11 - A._m11;

		float m12 = _m12 - A._m12;
		float m13 = _m13 - A._m13;
		float m14 = _m14 - A._m14;
		float m15 = _m15 - A._m15;

		Vec4 x(m0, m1, m2, m3);
		Vec4 y(m4, m5, m6, m7);
		Vec4 z(m8, m9, m10, m11);
		Vec4 w(m12, m13, m14, m15);

		return Mat4(x, y, z, w);
	}

	Mat4& Mat4::operator-=(const Mat4& A)
	{
		_m0 -= A._m0;
		_m1 -= A._m1;
		_m2 -= A._m2;
		_m3 -= A._m3;

		_m4 -= A._m4;
		_m5 -= A._m5;
		_m6 -= A._m6;
		_m7 -= A._m7;

		_m8 -= A._m8;
		_m9 -= A._m9;
		_m10 -= A._m10;
		_m11 -= A._m11;

		_m12 -= A._m12;
		_m13 -= A._m13;
		_m14 -= A._m14;
		_m15 -= A._m15;

		return *this;
	}

	Mat4 Mat4::operator*(const float s) const
	{
		// 0,1,2,3  
		// 4,5,6,7   
		// 8,9,10,11   
		// 12,13,14,15

		float x1 = _m0 * s;
		float x2 = _m1 * s;
		float x3 = _m2 * s;
		float x4 = _m3 * s;

		float y1 = _m4 * s;
		float y2 = _m5 * s;
		float y3 = _m6 * s;
		float y4 = _m7 * s;

		float z1 = _m8 * s;
		float z2 = _m9 * s;
		float z3 = _m10 * s;
		float z4 = _m11 * s;

		float w1 = _m12 * s;
		float w2 = _m13 * s;
		float w3 = _m14 * s;
		float w4 = _m15 * s;

		Vec4 x(x1, x2, x3, x4);
		Vec4 y(y1, y2, y3, y4);
		Vec4 z(z1, z2, z3, z4);
		Vec4 w(w1, w2, w3, w4);

		return Mat4(x, y, z, w);
	}

	Mat4 operator*(const float scale, const Mat4& A)
	{
		// 0,1,2,3  
		// 4,5,6,7   
		// 8,9,10,11   
		// 12,13,14,15

		float x1 = A._m0 * scale;
		float x2 = A._m1 * scale;
		float x3 = A._m2 * scale;
		float x4 = A._m3 * scale;

		float y1 = A._m4 * scale;
		float y2 = A._m5 * scale;
		float y3 = A._m6 * scale;
		float y4 = A._m7 * scale;

		float z1 = A._m8 * scale;
		float z2 = A._m9 * scale;
		float z3 = A._m10 * scale;
		float z4 = A._m11 * scale;

		float w1 = A._m12 * scale;
		float w2 = A._m13 * scale;
		float w3 = A._m14 * scale;
		float w4 = A._m15 * scale;

		Vec4 x(x1, x2, x3, x4);
		Vec4 y(y1, y2, y3, y4);
		Vec4 z(z1, z2, z3, z4);
		Vec4 w(w1, w2, w3, w4);

		return Mat4(x, y, z, w);
	}

	Mat4& Mat4::operator*=(const float scale)
	{
		// 0,1,2,3  
		// 4,5,6,7   
		// 8,9,10,11   
		// 12,13,14,15

		_m0 *= scale;
		_m1 *= scale;
		_m2 *= scale;
		_m3 *= scale;

		_m4 *= scale;
		_m5 *= scale;
		_m6 *= scale;
		_m7 *= scale;

		_m8 *= scale;
		_m9 *= scale;
		_m10 *= scale;
		_m11 *= scale;

		_m12 *= scale;
		_m13 *= scale;
		_m14 *= scale;
		_m15 *= scale;

		return *this;
	}

	Mat4 Mat4::operator*(const Mat4& A) const
	{
		// 0,1,2,3     0,4,8,12
		// 0,1,2,3     1,5,9,13
		// 0,1,2,3     2,6,10,14
		// 0,1,2,3     3,7,11,15
		float x1 = _m0 * A._m0 + _m1 * A._m4 + _m2 * A._m8 + _m3 * A._m12;
		float x2 = _m0 * A._m1 + _m1 * A._m5 + _m2 * A._m9 + _m3 * A._m13;
		float x3 = _m0 * A._m2 + _m1 * A._m6 + _m2 * A._m10 + _m3 * A._m14;
		float x4 = _m0 * A._m3 + _m1 * A._m7 + _m2 * A._m11 + _m3 * A._m15;

		// 4,5,6,7     0,4,8,12
		// 4,5,6,7     1,5,9,13
		// 4,5,6,7     2,6,10,14
		// 4,5,6,7     3,7,11,15
		float y1 = _m4 * A._m0 + _m5 * A._m4 + _m6 * A._m8 + _m7 * A._m12;
		float y2 = _m4 * A._m1 + _m5 * A._m5 + _m6 * A._m9 + _m7 * A._m13;
		float y3 = _m4 * A._m2 + _m5 * A._m6 + _m6 * A._m10 + _m7 * A._m14;
		float y4 = _m4 * A._m3 + _m5 * A._m7 + _m6 * A._m11 + _m7 * A._m15;

		// 8,9,10,11     0,4,8,12
		// 8,9,10,11     1,5,9,13
		// 8,9,10,11     2,6,10,14
		// 8,9,10,11     3,7,11,15
		float z1 = _m8 * A._m0 + _m9 * A._m4 + _m10 * A._m8 + _m11 * A._m12;
		float z2 = _m8 * A._m1 + _m9 * A._m5 + _m10 * A._m9 + _m11 * A._m13;
		float z3 = _m8 * A._m2 + _m9 * A._m6 + _m10 * A._m10 + _m11 * A._m14;
		float z4 = _m8 * A._m3 + _m9 * A._m7 + _m10 * A._m11 + _m11 * A._m15;

		// 12,13,14,15     0,4,8,12
		// 12,13,14,15     1,5,9,13
		// 12,13,14,15     2,6,10,14
		// 12,13,14,15     3,7,11,15
		float w1 = _m12 * A._m0 + _m13 * A._m4 + _m14 * A._m8 + _m15 * A._m12;
		float w2 = _m12 * A._m1 + _m13 * A._m5 + _m14 * A._m9 + _m15 * A._m13;
		float w3 = _m12 * A._m2 + _m13 * A._m6 + _m14 * A._m10 + _m15 * A._m14;
		float w4 = _m12 * A._m3 + _m13 * A._m7 + _m14 * A._m11 + _m15 * A._m15;

		Vec4 x(x1, x2, x3, x4);
		Vec4 y(y1, y2, y3, y4);
		Vec4 z(z1, z2, z3, z4);
		Vec4 w(w1, w2, w3, w4);

		Mat4 ret(x, y, z, w);
		ret.privSetNewHint(this->privGetHint(), A.privGetHint());

		/*Hint xd = ret.privGetHint();
		xd = xd;*/

		return ret;
		//return Mat4(x, y, z, w);
	}

	Mat4& Mat4::operator*=(const Mat4& A)
	{
		// 0,1,2,3     0,4,8,12
		// 0,1,2,3     1,5,9,13
		// 0,1,2,3     2,6,10,14
		// 0,1,2,3     3,7,11,15
		float x1 = _m0 * A._m0 + _m1 * A._m4 + _m2 * A._m8 + _m3 * A._m12;
		float x2 = _m0 * A._m1 + _m1 * A._m5 + _m2 * A._m9 + _m3 * A._m13;
		float x3 = _m0 * A._m2 + _m1 * A._m6 + _m2 * A._m10 + _m3 * A._m14;
		float x4 = _m0 * A._m3 + _m1 * A._m7 + _m2 * A._m11 + _m3 * A._m15;

		// 4,5,6,7     0,4,8,12
		// 4,5,6,7     1,5,9,13
		// 4,5,6,7     2,6,10,14
		// 4,5,6,7     3,7,11,15
		float y1 = _m4 * A._m0 + _m5 * A._m4 + _m6 * A._m8 + _m7 * A._m12;
		float y2 = _m4 * A._m1 + _m5 * A._m5 + _m6 * A._m9 + _m7 * A._m13;
		float y3 = _m4 * A._m2 + _m5 * A._m6 + _m6 * A._m10 + _m7 * A._m14;
		float y4 = _m4 * A._m3 + _m5 * A._m7 + _m6 * A._m11 + _m7 * A._m15;

		// 8,9,10,11     0,4,8,12
		// 8,9,10,11     1,5,9,13
		// 8,9,10,11     2,6,10,14
		// 8,9,10,11     3,7,11,15
		float z1 = _m8 * A._m0 + _m9 * A._m4 + _m10 * A._m8 + _m11 * A._m12;
		float z2 = _m8 * A._m1 + _m9 * A._m5 + _m10 * A._m9 + _m11 * A._m13;
		float z3 = _m8 * A._m2 + _m9 * A._m6 + _m10 * A._m10 + _m11 * A._m14;
		float z4 = _m8 * A._m3 + _m9 * A._m7 + _m10 * A._m11 + _m11 * A._m15;

		// 12,13,14,15     0,4,8,12
		// 12,13,14,15     1,5,9,13
		// 12,13,14,15     2,6,10,14
		// 12,13,14,15     3,7,11,15
		float w1 = _m12 * A._m0 + _m13 * A._m4 + _m14 * A._m8 + _m15 * A._m12;
		float w2 = _m12 * A._m1 + _m13 * A._m5 + _m14 * A._m9 + _m15 * A._m13;
		float w3 = _m12 * A._m2 + _m13 * A._m6 + _m14 * A._m10 + _m15 * A._m14;
		float w4 = _m12 * A._m3 + _m13 * A._m7 + _m14 * A._m11 + _m15 * A._m15;

		_v0 = Vec4(x1, x2, x3, x4);
		_v1 = Vec4(y1, y2, y3, y4);
		_v2 = Vec4(z1, z2, z3, z4);
		_v3 = Vec4(w1, w2, w3, w4);

		return *this;
	}

	Mat4 Mat4::operator*(const Quat& q) const
	{
		return *this * Mat4(q);
	}

	Mat4& Mat4::operator*=(const Quat& q)
	{
		*this *= Mat4(q);

		return *this;
	}

	Mat4 Mat4::operator*(const Scale& A) const
	{
		if (this->privGetHint() > Hint::Generalize)
		{
			Mat4 ret = Mat4(Vec4(this->_m0 * A._m0, this->_m1 * A._m5, this->_m2 * A._m10, this->_m3),
							Vec4(this->_m4 * A._m0, this->_m5 * A._m5, this->_m6 * A._m10, this->_m7),
							Vec4(this->_m8 * A._m0, this->_m9 * A._m5, this->_m10 * A._m10, this->_m11),
							Vec4(this->_m12 * A._m0, this->_m13 * A._m5, this->_m14 * A._m10, this->_m15));


			ret.privSetNewHint(this->privGetHint(), Hint::Scale);
			
			return ret;
		}
		else
		{
			return Mat4(Vec4(this->_m0 * A._m0, this->_m1 * A._m5, this->_m2 * A._m10, this->_m3),
						Vec4(this->_m4 * A._m0, this->_m5 * A._m5, this->_m6 * A._m10, this->_m7),
						Vec4(this->_m8 * A._m0, this->_m9 * A._m5, this->_m10 * A._m10, this->_m11),
						Vec4(this->_m12 * A._m0, this->_m13 * A._m5, this->_m14 * A._m10, this->_m15));
		}
	}

	Mat4& Mat4::operator*=(const Scale& A)
	{
		Hint prevHint = this->privGetHint();

		this->_m0 = this->_m0 * A._m0;
		this->_m1 = this->_m1 * A._m5;
		this->_m2 = this->_m2 * A._m10;

		this->_m4 = this->_m4 * A._m0;
		this->_m5 = this->_m5 * A._m5;
		this->_m6 = this->_m6 * A._m10;

		this->_m8 = this->_m8 * A._m0;
		this->_m9 = this->_m9 * A._m5;
		this->_m10 = this->_m10 * A._m10;

		this->_m12 = this->_m12 * A._m0;
		this->_m13 = this->_m13 * A._m5;
		this->_m14 = this->_m14 * A._m10;

		this->privSetNewHint(prevHint, Hint::Scale);

		return *this;
	}

	Mat4 Mat4::operator*(const Rot& A) const
	{
		if (this->privGetHint() > Hint::Generalize)
		{
			Mat4 ret = Mat4(Vec4(this->_m0 * A._m0 + this->_m1 * A._m4 + this->_m2 * A._m8,
								 this->_m0 * A._m1 + this->_m1 * A._m5 + this->_m2 * A._m9,
								 this->_m0 * A._m2 + this->_m1 * A._m6 + this->_m2 * A._m10,
								 this->_m3),

							Vec4(this->_m4 * A._m0 + this->_m5 * A._m4 + this->_m6 * A._m8,
								 this->_m4 * A._m1 + this->_m5 * A._m5 + this->_m6 * A._m9,
								 this->_m4 * A._m2 + this->_m5 * A._m6 + this->_m6 * A._m10,
								 this->_m7),

							Vec4(this->_m8 * A._m0 + this->_m9 * A._m4 + this->_m10 * A._m8,
								 this->_m8 * A._m1 + this->_m9 * A._m5 + this->_m10 * A._m9,
								 this->_m8 * A._m2 + this->_m9 * A._m6 + this->_m10 * A._m10,
								 this->_m11),

							Vec4(this->_m12 * A._m0 + this->_m13 * A._m4 + this->_m14 * A._m8,
								 this->_m12 * A._m1 + this->_m13 * A._m5 + this->_m14 * A._m9,
								 this->_m12 * A._m2 + this->_m13 * A._m6 + this->_m14 * A._m10,
								 this->_m15));

			ret.privSetNewHint(this->privGetHint(), Hint::Rot);

			return ret;
		}
		else
		{
			return Mat4(Vec4(this->_m0 * A._m0 + this->_m1 * A._m4 + this->_m2 * A._m8,
							 this->_m0 * A._m1 + this->_m1 * A._m5 + this->_m2 * A._m9,
							 this->_m0 * A._m2 + this->_m1 * A._m6 + this->_m2 * A._m10,
							 this->_m3),

						Vec4(this->_m4 * A._m0 + this->_m5 * A._m4 + this->_m6 * A._m8,
							 this->_m4 * A._m1 + this->_m5 * A._m5 + this->_m6 * A._m9,
							 this->_m4 * A._m2 + this->_m5 * A._m6 + this->_m6 * A._m10,
							 this->_m7),

						Vec4(this->_m8 * A._m0 + this->_m9 * A._m4 + this->_m10 * A._m8,
							 this->_m8 * A._m1 + this->_m9 * A._m5 + this->_m10 * A._m9,
							 this->_m8 * A._m2 + this->_m9 * A._m6 + this->_m10 * A._m10,
							 this->_m11),

						Vec4(this->_m12 * A._m0 + this->_m13 * A._m4 + this->_m14 * A._m8,
							 this->_m12 * A._m1 + this->_m13 * A._m5 + this->_m14 * A._m9,
							 this->_m12 * A._m2 + this->_m13 * A._m6 + this->_m14 * A._m10,
							 this->_m15));
		}
	}

	Mat4& Mat4::operator*=(const Rot& A)
	{
		Hint prevHint = this->privGetHint();

		float new_m0 = this->_m0 * A._m0 + this->_m1 * A._m4 + this->_m2 * A._m8;
		float new_m1 = this->_m0 * A._m1 + this->_m1 * A._m5 + this->_m2 * A._m9;
		float new_m2 = this->_m0 * A._m2 + this->_m1 * A._m6 + this->_m2 * A._m10;

		float new_m4 = this->_m4 * A._m0 + this->_m5 * A._m4 + this->_m6 * A._m8;
		float new_m5 = this->_m4 * A._m1 + this->_m5 * A._m5 + this->_m6 * A._m9;
		float new_m6 = this->_m4 * A._m2 + this->_m5 * A._m6 + this->_m6 * A._m10;

		float new_m8 = this->_m8 * A._m0 + this->_m9 * A._m4 + this->_m10 * A._m8;
		float new_m9 = this->_m8 * A._m1 + this->_m9 * A._m5 + this->_m10 * A._m9;
		float new_m10 = this->_m8 * A._m2 + this->_m9 * A._m6 + this->_m10 * A._m10;

		float new_m12 = this->_m12 * A._m0 + this->_m13 * A._m4 + this->_m14 * A._m8;
		float new_m13 = this->_m12 * A._m1 + this->_m13 * A._m5 + this->_m14 * A._m9;
		float new_m14 = this->_m12 * A._m2 + this->_m13 * A._m6 + this->_m14 * A._m10;

		this->_m0 = new_m0;
		this->_m1 = new_m1;
		this->_m2 = new_m2;

		this->_m4 = new_m4;
		this->_m5 = new_m5;
		this->_m6 = new_m6;

		this->_m8 = new_m8;
		this->_m9 = new_m9;
		this->_m10 = new_m10;

		this->_m12 = new_m12;
		this->_m13 = new_m13;
		this->_m14 = new_m14;

		this->privSetNewHint(prevHint, Hint::Rot);

		return *this;
	}

	Mat4 Mat4::operator*(const Trans& A) const
	{
		if (this->privGetHint() > Hint::Generalize)
		{
			Mat4 ret = Mat4(Vec4(this->_m0 + this->_m3 * A._m12, this->_m1 + this->_m3 * A._m13, this->_m2 + this->_m3 * A._m14, this->_m3),
							Vec4(this->_m4 + this->_m7 * A._m12, this->_m5 + this->_m7 * A._m13, this->_m6 + this->_m7 * A._m14, this->_m7),
							Vec4(this->_m8 + this->_m11 * A._m12, this->_m9 + this->_m11 * A._m13, this->_m10 + this->_m11 * A._m14, this->_m11),
							Vec4(this->_m12 + this->_m15 * A._m12, this->_m13 + this->_m15 * A._m13, this->_m14 + this->_m15 * A._m14, this->_m15));

			ret.privSetNewHint(this->privGetHint(), Hint::Trans);

			return ret;
		}
		else
		{
			return Mat4(Vec4(this->_m0 + this->_m3 * A._m12, this->_m1 + this->_m3 * A._m13, this->_m2 + this->_m3 * A._m14, this->_m3),
					    Vec4(this->_m4 + this->_m7 * A._m12, this->_m5 + this->_m7 * A._m13, this->_m6 + this->_m7 * A._m14, this->_m7),
				        Vec4(this->_m8 + this->_m11 * A._m12, this->_m9 + this->_m11 * A._m13, this->_m10 + this->_m11 * A._m14, this->_m11),
				        Vec4(this->_m12 + this->_m15 * A._m12, this->_m13 + this->_m15 * A._m13, this->_m14 + this->_m15 * A._m14, this->_m15));
		}
	}

	Mat4& Mat4::operator*=(const Trans& A)
	{
		Hint prevHint = this->privGetHint();

		this->_m0 = this->_m0 + this->_m3 * A._m12;
		this->_m1 = this->_m1 + this->_m3 * A._m13;
		this->_m2 = this->_m2 + this->_m3 * A._m14;

		this->_m4 = this->_m4 + this->_m7 * A._m12;
		this->_m5 = this->_m5 + this->_m7 * A._m13;
		this->_m6 = this->_m6 + this->_m7 * A._m14;

		this->_m8 = this->_m8 + this->_m11 * A._m12;
		this->_m9 = this->_m9 + this->_m11 * A._m13;
		this->_m10 = this->_m10 + this->_m11 * A._m14;

		this->_m12 = this->_m12 + this->_m15 * A._m12;
		this->_m13 = this->_m13 + this->_m15 * A._m13;
		this->_m14 = this->_m14 + this->_m15 * A._m14;

		this->privSetNewHint(prevHint, Hint::Trans);

		return *this;
	}

	bool Mat4::privHasHint() const
	{
		return ((_u_m15 & 0xFFFFFFF0) == 0x3f800000);
	}

	/*const bool Mat4::isEqual(const Mat4& A, const float epsilon) const
	{
		return false;
	}

	const bool Mat4::isIdentity(const float epsilon) const
	{
		return false;
	}*/

	inline Mat4::Hint Mat4::privGetHint() const
	{
		// bitwise & to check the last 3 bits with 7 - 0111
		unsigned int mask = 7;
		unsigned int checkHint = this->_u_m15;

		Hint hint = Hint(checkHint & mask);
		
		return hint;
	}
}