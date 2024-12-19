#include "MathEngine.h"

namespace EngineCore
{
	Trans::Trans()
	{
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

		this->privSetTransHint();
	}

	Trans::Trans(const float tx, const float ty, const float tz)
	{
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

		this->_m12 = tx;
		this->_m13 = ty;
		this->_m14 = tz;
		this->_m15 = 1.0f;

		// Set translation hint
		this->privSetTransHint();
	}

	Trans::Trans(const Vec3& vTrans)
	{
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

		this->_m12 = vTrans.x();
		this->_m13 = vTrans.y();
		this->_m14 = vTrans.z();
		this->_m15 = 1.0f;

		// Set translation hint
		this->privSetTransHint();
	}

	void Trans::set(const float tx, const float ty, const float tz)
	{
		this->_m12 = tx;
		this->_m13 = ty;
		this->_m14 = tz;

		// Set translation hint
		this->privSetTransHint();
	}

	void Trans::set(const Vec3& vTrans)
	{
		this->_m12 = vTrans.x();
		this->_m13 = vTrans.y();
		this->_m14 = vTrans.z();

		// Set translation hint
		this->privSetTransHint();
	}

	Mat4 Trans::operator*(const Mat4& A) const
	{
		return Mat4(Vec4(A._m0, A._m1, A._m2, A._m3),
					Vec4(A._m4, A._m5, A._m6, A._m7),
					Vec4(A._m8, A._m9, A._m10, A._m11),
					Vec4(this->_m12 * A._m0 + this->_m13 * A._m4 + this->_m14 * A._m8 + this->_m15 * A._m12,
						 this->_m12 * A._m1 + this->_m13 * A._m5 + this->_m14 * A._m9 + this->_m15 * A._m13,
						 this->_m12 * A._m2 + this->_m13 * A._m6 + this->_m14 * A._m10 + this->_m15 * A._m14,
						 this->_m12 * A._m3 + this->_m13 * A._m7 + this->_m14 * A._m11 + this->_m15 * A._m15));
	}

	Mat4 Trans::operator*(const Quat& q) const
	{
		return *this * Mat4(q);
	}

	Mat4 Trans::operator*(const Scale& A) const
	{
		Vec4 x(A._m0, this->_m1, this->_m2, this->_m3);
		Vec4 y(this->_m4, A._m5, this->_m6, this->_m7);
		Vec4 z(this->_m8, this->_m9, A._m10, this->_m11);
		Vec4 w(this->_m12 * A._m0, this->_m13 * A._m5, this->_m14 * A._m10, this->_m15);

		Mat4 ret(x, y, z, w);
		ret.privSetNewHint(Mat4::Hint::Trans, Mat4::Hint::Scale);

		return ret;
	}

	Mat4 Trans::operator*(const Rot& A) const
	{
		Vec4 x(A._m0, A._m1, A._m2, this->_m3);
		Vec4 y(A._m4, A._m5, A._m6, this->_m7);
		Vec4 z(A._m8, A._m9, A._m10, this->_m11);
		Vec4 w(this->_m12 * A._m0 + this->_m13 * A._m4 + this->_m14 * A._m8, 
			   this->_m12 * A._m1 + this->_m13 * A._m5 + this->_m14 * A._m9,
			   this->_m12 * A._m2 + this->_m13 * A._m6 + this->_m14 * A._m10,
			   this->_m15);

		Mat4 ret(x, y, z, w);
		ret.privSetNewHint(Mat4::Hint::Trans, Mat4::Hint::Rot);

		return ret;
	}

	Trans Trans::operator*(const Trans& A) const
	{
		return Trans(this->_m12 + A._m12, this->_m13 + A._m13, this->_m14 + A._m14);
	}

	Trans& Trans::operator*=(const Trans& A)
	{
		this->_m12 = this->_m12 + A._m12;
		this->_m13 = this->_m13 + A._m13;
		this->_m14 = this->_m14 + A._m14;
		
		return *this;
	}

}