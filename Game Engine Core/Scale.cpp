#include "MathEngine.h"

namespace EngineCore
{
	Scale::Scale()
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

		// Set scale hint
		this->privSetScaleHint();
	}

	Scale::Scale(const float sx, const float sy, const float sz)
	{
		this->_m0 = sx;
		this->_m1 = 0.0f;
		this->_m2 = 0.0f;
		this->_m3 = 0.0f;

		this->_m4 = 0.0f;
		this->_m5 = sy;
		this->_m6 = 0.0f;
		this->_m7 = 0.0f;

		this->_m8 = 0.0f;
		this->_m9 = 0.0f;
		this->_m10 = sz;
		this->_m11 = 0.0f;

		this->_m12 = 0.0f;
		this->_m13 = 0.0f;
		this->_m14 = 0.0f;
		this->_m15 = 1.0f;

		// Set scale hint
		this->privSetScaleHint();
	}

	Scale::Scale(const Vec3& vScale)
	{
		this->_m0 = vScale.x();
		this->_m1 = 0.0f;
		this->_m2 = 0.0f;
		this->_m3 = 0.0f;

		this->_m4 = 0.0f;
		this->_m5 = vScale.y();
		this->_m6 = 0.0f;
		this->_m7 = 0.0f;

		this->_m8 = 0.0f;
		this->_m9 = 0.0f;
		this->_m10 = vScale.z();
		this->_m11 = 0.0f;

		this->_m12 = 0.0f;
		this->_m13 = 0.0f;
		this->_m14 = 0.0f;
		this->_m15 = 1.0f;

		// Set scale hint
		this->privSetScaleHint();
	}

	void Scale::set(const float sx, const float sy, const float sz)
	{
		this->_m0 = sx;
		this->_m5 = sy;
		this->_m10 = sz;

		// Set scale hint
		this->privSetScaleHint();
	}

	void Scale::set(const Vec3& vScale)
	{
		this->_m0 = vScale.x();
		this->_m5 = vScale.y();
		this->_m10 = vScale.z();

		// Set scale hint
		this->privSetScaleHint();
	}

	Mat4 Scale::operator*(const Mat4& A) const
	{
		return Mat4(Vec4(A._m0 * this->_m0, A._m1 * this->_m0, A._m2 * this->_m0, A._m3 * this->_m0),
					Vec4(A._m4 * this->_m5, A._m5 * this->_m5, A._m6 * this->_m5, A._m7 * this->_m5),
					Vec4(A._m8 * this->_m10, A._m9 * this->_m10, A._m10 * this->_m10, A._m11 * this->_m10),
					Vec4(A._m12, A._m13, A._m14, A._m15));
	}

	Mat4 Scale::operator*(const Quat& q) const
	{
		return *this * Mat4(q);
	}

	Scale Scale::operator*(const Scale& A) const
	{
		return Scale(this->_m0 * A._m0, this->_m5 * A._m5, this->_m10 * A._m10);
	}

	Scale& Scale::operator*=(const Scale& A)
	{
		this->_m0 = this->_m0 * A._m0;
		this->_m5 = this->_m5 * A._m5;
		this->_m10 = this->_m10 * A._m10;

		this->privSetScaleHint();

		return *this;
	}

	Mat4 Scale::operator*(const Rot& A) const
	{
		Vec4 x(this->_m0 * A._m0, this->_m0 * A._m1, this->_m0 * A._m2, this->_m3);
		Vec4 y(this->_m5 * A._m4, this->_m5 * A._m5, this->_m5 * A._m6, this->_m7);
		Vec4 z(this->_m10 * A._m8, this->_m10 * A._m9, this->_m10 * A._m10, this->_m11);
		Vec4 w(this->_m12, this->_m13, this->_m14, this->_m15);

		Mat4 ret(x, y, z, w);
		ret.privSetNewHint(Mat4::Hint::Scale, Mat4::Hint::Rot);

		return ret;
	}

	Mat4 Scale::operator*(const Trans& A) const
	{
		Vec4 x(this->_m0, this->_m1, this->_m2, this->_m3);
		Vec4 y(this->_m4, this->_m5, this->_m6, this->_m7);
		Vec4 z(this->_m8, this->_m9, this->_m10, this->_m11);
		Vec4 w(A._m12, A._m13, A._m14, this->_m15);

		Mat4 ret(x, y, z ,w);
		ret.privSetNewHint(Mat4::Hint::Scale, Mat4::Hint::Trans);
	
		return ret;
	}
}