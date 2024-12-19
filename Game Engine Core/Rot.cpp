#include "MathEngine.h"
#include <cassert>

namespace EngineCore
{
	Rot::Rot()
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

		// Set rotation hint
		this->privSetRotHint();
	}

	Rot& Rot::operator=(const Quat &q)
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

		this->privSetRotHint();

		return *this;
	}

	Rot::Rot(const Special type)
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

	Rot::Rot(const Quat &q)
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

		this->privSetRotHint();
	}

	void Rot::set(const Quat &q)
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

		this->privSetRotHint();
	}

	void Rot::set(const Special type)
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

	Mat4 Rot::operator*(const Mat4& A) const
	{
		return Mat4(Vec4(this->_m0 * A._m0 + this->_m1 * A._m4 + this->_m2 * A._m8, 
						 this->_m0 * A._m1 + this->_m1 * A._m5 + this->_m2 * A._m9,
					     this->_m0 * A._m2 + this->_m1 * A._m6 + this->_m2 * A._m10,
						 this->_m0 * A._m3 + this->_m1 * A._m7 + this->_m2 * A._m11),

					Vec4(this->_m4 * A._m0 + this->_m5 * A._m4 + this->_m6 * A._m8,
						 this->_m4 * A._m1 + this->_m5 * A._m5 + this->_m6 * A._m9,
						 this->_m4 * A._m2 + this->_m5 * A._m6 + this->_m6 * A._m10,
						 this->_m4 * A._m3 + this->_m5 * A._m7 + this->_m6 * A._m11),

					Vec4(this->_m8 * A._m0 + this->_m9 * A._m4 + this->_m10 * A._m8,
						 this->_m8 * A._m1 + this->_m9 * A._m5 + this->_m10 * A._m9,
						 this->_m8 * A._m2 + this->_m9 * A._m6 + this->_m10 * A._m10,
						 this->_m8 * A._m3 + this->_m9 * A._m7 + this->_m10 * A._m11),

					Vec4(A._m12, A._m13, A._m14, A._m15));
	}

	Rot Rot::operator*(const Quat &q) const
	{
		Rot ret = Rot();

		float new_m0 = this->_m0 * (1.0f - 2.0f * (q._qy * q._qy + q._qz * q._qz)) + this->_m1 * (2.0f * (q._qx * q._qy - q._qw * q._qz)) + this->_m2 * (2.0f * (q._qx * q._qz + q._qw * q._qy));
		float new_m1 = this->_m0 * (2.0f * (q._qx * q._qy + q._qw * q._qz)) + this->_m1 * (1.0f - 2.0f * (q._qx * q._qx + q._qz * q._qz)) + this->_m2 * (2.0f * (q._qy * q._qz - q._qw * q._qx));
		float new_m2 = this->_m0 * (2.0f * (q._qx * q._qz - q._qw * q._qy)) + this->_m1 * (2.0f * (q._qy * q._qz + q._qw * q._qx)) + this->_m2 * (1.0f - 2.0f * (q._qx * q._qx + q._qy * q._qy));

		float new_m4 = this->_m4 * (1.0f - 2.0f * (q._qy * q._qy + q._qz * q._qz)) + this->_m5 * (2.0f * (q._qx * q._qy - q._qw * q._qz)) + this->_m6 * (2.0f * (q._qx * q._qz + q._qw * q._qy));
		float new_m5 = this->_m4 * (2.0f * (q._qx * q._qy + q._qw * q._qz)) + this->_m5 * (1.0f - 2.0f * (q._qx * q._qx + q._qz * q._qz)) + this->_m6 * (2.0f * (q._qy * q._qz - q._qw * q._qx));
		float new_m6 = this->_m4 * (2.0f * (q._qx * q._qz - q._qw * q._qy)) + this->_m5 * (2.0f * (q._qy * q._qz + q._qw * q._qx)) + this->_m6 * (1.0f - 2.0f * (q._qx * q._qx + q._qy * q._qy));

		float new_m8 = this->_m8 * (1.0f - 2.0f * (q._qy * q._qy + q._qz * q._qz)) + this->_m9 * (2.0f * (q._qx * q._qy - q._qw * q._qz)) + this->_m10 * (2.0f * (q._qx * q._qz + q._qw * q._qy));
		float new_m9 = this->_m8 * (2.0f * (q._qx * q._qy + q._qw * q._qz)) + this->_m9 * (1.0f - 2.0f * (q._qx * q._qx + q._qz * q._qz)) + this->_m10 * (2.0f * (q._qy * q._qz - q._qw * q._qx));
		float new_m10 = this->_m8 * (2.0f * (q._qx * q._qz - q._qw * q._qy)) + this->_m9 * (2.0f * (q._qy * q._qz + q._qw * q._qx)) + this->_m10 * (1.0f - 2.0f * (q._qx * q._qx + q._qy * q._qy));

		ret._m0 = new_m0;
		ret._m1 = new_m1;
		ret._m2 = new_m2;

		ret._m4 = new_m4;
		ret._m5 = new_m5;
		ret._m6 = new_m6;

		ret._m8 = new_m8;
		ret._m9 = new_m9;
		ret._m10 = new_m10;

		ret.privSetRotHint();

		return ret;
	}

	Rot& Rot::operator*=(const Quat &q)
	{
		//  (1.0f - 2.0f * (q._qy * q._qy + q._qz * q._qz))   (2.0f * (q._qx * q._qy + q._qw * q._qz))           (2.0f * (q._qx * q._qz - q._qw * q._qy))

		//  (2.0f * (q._qx * q._qy - q._qw * q._qz))          (1.0f - 2.0f * (q._qx * q._qx + q._qz * q._qz))    (2.0f * (q._qy * q._qz + q._qw * q._qx))

		//  (2.0f * (q._qx * q._qz + q._qw * q._qy))          (2.0f * (q._qy * q._qz - q._qw * q._qx))           (1.0f - 2.0f * (q._qx * q._qx + q._qy * q._qy))

		float new_m0 = this->_m0 * (1.0f - 2.0f * (q._qy * q._qy + q._qz * q._qz)) + this->_m1 * (2.0f * (q._qx * q._qy - q._qw * q._qz)) + this->_m2 * (2.0f * (q._qx * q._qz + q._qw * q._qy));
		float new_m1 = this->_m0 * (2.0f * (q._qx * q._qy + q._qw * q._qz)) + this->_m1 * (1.0f - 2.0f * (q._qx * q._qx + q._qz * q._qz)) + this->_m2 * (2.0f * (q._qy * q._qz - q._qw * q._qx));
		float new_m2 = this->_m0 * (2.0f * (q._qx * q._qz - q._qw * q._qy)) + this->_m1 * (2.0f * (q._qy * q._qz + q._qw * q._qx)) + this->_m2 * (1.0f - 2.0f * (q._qx * q._qx + q._qy * q._qy));

		float new_m4 = this->_m4 * (1.0f - 2.0f * (q._qy * q._qy + q._qz * q._qz)) + this->_m5 * (2.0f * (q._qx * q._qy - q._qw * q._qz)) + this->_m6 * (2.0f * (q._qx * q._qz + q._qw * q._qy));
		float new_m5 = this->_m4 * (2.0f * (q._qx * q._qy + q._qw * q._qz)) + this->_m5 * (1.0f - 2.0f * (q._qx * q._qx + q._qz * q._qz)) + this->_m6 * (2.0f * (q._qy * q._qz - q._qw * q._qx));
		float new_m6 = this->_m4 * (2.0f * (q._qx * q._qz - q._qw * q._qy)) + this->_m5 * (2.0f * (q._qy * q._qz + q._qw * q._qx)) + this->_m6 * (1.0f - 2.0f * (q._qx * q._qx + q._qy * q._qy));

		float new_m8 = this->_m8 * (1.0f - 2.0f * (q._qy * q._qy + q._qz * q._qz)) + this->_m9 * (2.0f * (q._qx * q._qy - q._qw * q._qz)) + this->_m10 * (2.0f * (q._qx * q._qz + q._qw * q._qy));
		float new_m9 = this->_m8 * (2.0f * (q._qx * q._qy + q._qw * q._qz)) + this->_m9 * (1.0f - 2.0f * (q._qx * q._qx + q._qz * q._qz)) + this->_m10 * (2.0f * (q._qy * q._qz - q._qw * q._qx));
		float new_m10 = this->_m8 * (2.0f * (q._qx * q._qz - q._qw * q._qy)) + this->_m9 * (2.0f * (q._qy * q._qz + q._qw * q._qx)) + this->_m10 * (1.0f - 2.0f * (q._qx * q._qx + q._qy * q._qy));

		this->_m0 = new_m0;
		this->_m1 = new_m1;
		this->_m2 = new_m2;
		this->_m3 = 0.0f;
		this->_m4 = new_m4;
		this->_m5 = new_m5;
		this->_m6 = new_m6;
		this->_m7 = 0.0f;
		this->_m8 = new_m8;
		this->_m9 = new_m9;
		this->_m10 = new_m10;
		this->_m11 = 0.0f;
		this->_m12 = 0.0f;
		this->_m13 = 0.0f;
		this->_m14 = 0.0f;
		this->_m15 = 1.0f;

		this->privSetRotHint();

		return *this;
	}

	Mat4 Rot::operator*(const Scale& A) const
	{
		Vec4 x(this->_m0 * A._m0, this->_m1 * A._m5, this->_m2 * A._m10, 0.0f);
		Vec4 y(this->_m4 * A._m0, this->_m5 * A._m5, this->_m6 * A._m10, 0.0f);
		Vec4 z(this->_m8 * A._m0, this->_m9 * A._m5, this->_m10 * A._m10, 0.0f);
		Vec4 w(A._m12, A._m13, A._m14, A._m15);

		Mat4 ret(x, y, z, w);
		ret.privSetNewHint(Mat4::Hint::Rot, Mat4::Hint::Scale);

		return ret;
	}

	Rot Rot::operator*(const Rot& A) const
	{
		Rot ret = Rot();

		ret._m0 = this->_m0 * A._m0 + this->_m1 * A._m4 + this->_m2 * A._m8;
		ret._m1 = this->_m0 * A._m1 + this->_m1 * A._m5 + this->_m2 * A._m9;
		ret._m2 = this->_m0 * A._m2 + this->_m1 * A._m6 + this->_m2 * A._m10;

		ret._m4 = this->_m4 * A._m0 + this->_m5 * A._m4 + this->_m6 * A._m8;
		ret._m5 = this->_m4 * A._m1 + this->_m5 * A._m5 + this->_m6 * A._m9;
		ret._m6 = this->_m4 * A._m2 + this->_m5 * A._m6 + this->_m6 * A._m10;

		ret._m8 = this->_m8 * A._m0 + this->_m9 * A._m4 + this->_m10 * A._m8;
		ret._m9 = this->_m8 * A._m1 + this->_m9 * A._m5 + this->_m10 * A._m9;
		ret._m10 = this->_m8 * A._m2 + this->_m9 * A._m6 + this->_m10 * A._m10;
			
		return ret;
	}

	Rot& Rot::operator*=(const Rot& A)
	{
		float new_m0 = this->_m0 * A._m0 + this->_m1 * A._m4 + this->_m2 * A._m8;
		float new_m1 = this->_m0 * A._m1 + this->_m1 * A._m5 + this->_m2 * A._m9;
		float new_m2 = this->_m0 * A._m2 + this->_m1 * A._m6 + this->_m2 * A._m10;

		float new_m4 = this->_m4 * A._m0 + this->_m5 * A._m4 + this->_m6 * A._m8;
		float new_m5 = this->_m4 * A._m1 + this->_m5 * A._m5 + this->_m6 * A._m9;
		float new_m6 = this->_m4 * A._m2 + this->_m5 * A._m6 + this->_m6 * A._m10;

		float new_m8 = this->_m8 * A._m0 + this->_m9 * A._m4 + this->_m10 * A._m8;
		float new_m9 = this->_m8 * A._m1 + this->_m9 * A._m5 + this->_m10 * A._m9;
		float new_m10 = this->_m8 * A._m2 + this->_m9 * A._m6 + this->_m10 * A._m10;

		this->_m0 = new_m0;
		this->_m1 = new_m1;
		this->_m2 = new_m2;

		this->_m4 = new_m4;
		this->_m5 = new_m5;
		this->_m6 = new_m6;

		this->_m8 = new_m8;
		this->_m9 = new_m9;
		this->_m10 = new_m10;

		return *this;
	}

	Mat4 Rot::operator*(const Trans& A) const
	{
		Vec4 x(this->_m0, this->_m1, this->_m2, 0.0f);
		Vec4 y(this->_m4, this->_m5, this->_m6, 0.0f);
		Vec4 z(this->_m8, this->_m9, this->_m10, 0.0f);
		Vec4 w(A._m12, A._m13, A._m14, A._m15);

		Mat4 ret(x, y, z, w);
		ret.privSetNewHint(Mat4::Hint::Rot, Mat4::Hint::Trans);

		return ret;
	}

	Rot::Rot(const Rot1 type, const float angle)
	{
		switch (type)
		{
		case Rot1::X:
			this->_m0 = 1.0f;
			this->_m1 = 0.0f;
			this->_m2 = 0.0f;
			this->_m3 = 0.0f;
			this->_m4 = 0.0f;
			this->_m5 = Trig::cos(angle);
			this->_m6 = Trig::sin(angle);
			this->_m7 = 0.0f;
			this->_m8 = 0.0f;
			this->_m9 = -Trig::sin(angle);
			this->_m10 = Trig::cos(angle);
			this->_m11 = 0.0f;
			this->_m12 = 0.0f;
			this->_m13 = 0.0f;
			this->_m14 = 0.0f;
			this->_m15 = 1.0f;
			break;
		case Rot1::Y:
			this->_m0 = Trig::cos(angle);
			this->_m1 = 0.0f;
			this->_m2 = -Trig::sin(angle);
			this->_m3 = 0.0f;
			this->_m4 = 0.0f;
			this->_m5 = 1.0f;
			this->_m6 = 0.0f;
			this->_m7 = 0.0f;
			this->_m8 = Trig::sin(angle);
			this->_m9 = 0.0f;
			this->_m10 = Trig::cos(angle);
			this->_m11 = 0.0f;
			this->_m12 = 0.0f;
			this->_m13 = 0.0f;
			this->_m14 = 0.0f;
			this->_m15 = 1.0f;
			break;
		case Rot1::Z:
			this->_m0 = Trig::cos(angle);
			this->_m1 = Trig::sin(angle);
			this->_m2 = 0.0f;
			this->_m3 = 0.0f;
			this->_m4 = -Trig::sin(angle);
			this->_m5 = Trig::cos(angle);
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
		}

		// Set rotation hint
		this->privSetRotHint();
	}

	Rot::Rot(const Rot3 mode, const float angle_0, const float angle_1, const float angle_2)
	{
		Rot X = Rot(Rot1::X, angle_0);
		Rot Y = Rot(Rot1::Y, angle_1);
		Rot Z = Rot(Rot1::Z, angle_2);

		switch (mode)
		{
		case Rot3::XYZ:
			Mat4::set(X * Y * Z);
			break;
		case Rot3::XZY:
			Mat4::set(X * Z * Y);
			break;
		case Rot3::YXZ:
			Mat4::set(Y * X * Z);
			break;
		case Rot3::YZX:
			Mat4::set(Y * Z * X);
			break;
		case Rot3::ZXY:
			Mat4::set(Z * X * Y);
			break;
		case Rot3::ZYX:
			Mat4::set(Z * Y * X);
			break;
		}

		// Set rotation hint
		this->privSetRotHint();
	}

	Rot::Rot(const Axis mode, const Vec3& vAxis, const float angle_radians)
	{
		const float angle_a = 0.5f * angle_radians;
		float cos_a;
		float sin_a;

		Trig::cossin(cos_a, sin_a, angle_a);

		Vec3 qV;

		switch (mode)
		{
		case Axis::AxisAngle:
			qV = vAxis.getNorm();
			break;
		case Axis::UnitAxisAngle:
			qV = vAxis;
			break;
		}

		qV *= sin_a;

		Vec4 Q;
		Q[x] = qV[x];
		Q[y] = qV[y];
		Q[z] = qV[z];
		Q[w] = cos_a;

		// this function has been transposed
		float x2, y2, z2;
		float xx, xy, xz;
		float yy, yz, zz;
		float wx, wy, wz;

		// ADD test to make sure that quat is normalized

		x2 = Q[x] + Q[x];
		y2 = Q[y] + Q[y];
		z2 = Q[z] + Q[z];

		xx = Q[x] * x2;
		xy = Q[x] * y2;
		xz = Q[x] * z2;

		yy = Q[y] * y2;
		yz = Q[y] * z2;
		zz = Q[z] * z2;

		wx = Q[w] * x2;
		wy = Q[w] * y2;
		wz = Q[w] * z2;

		this->_m0 = 1.0f - (yy + zz);
		this->_m1 = xy + wz;
		this->_m2 = xz - wy;
		this->_m3 = 0.0f;

		this->_m4 = xy - wz;
		this->_m5 = 1.0f - (xx + zz);
		this->_m6 = yz + wx;
		this->_m7 = 0.0f;

		this->_m8 = xz + wy;
		this->_m9 = yz - wx;
		this->_m10 = 1.0f - (xx + yy);
		this->_m11 = 0.0f;

		this->_v3.set(0.0f, 0.0f, 0.0f, 1.0f);

		// Set rotation hint
		this->privSetRotHint();
	}

	Rot::Rot(const Orient type, const Vec3& dof, const Vec3& up)
	{
		/* make sure the DOF and VUP are not parallel */
		assert(dof.dot(up) != 1.0f);

		/* rz = vect_dof */
		Vec3 rz = dof.getNorm();

		/* find rx */
		Vec3 rx = up.cross(rz);
		rx.norm();

		/* find ry */
		Vec3 ry = rz.cross(rx);
		ry.norm();

		Mat4::set(Row4::i0, Vec4(rx, 0));
		Mat4::set(Row4::i1, Vec4(ry, 0));
		Mat4::set(Row4::i2, Vec4(rz, 0));
		Mat4::set(Row4::i3, Vec4(0.0f, 0.0f, 0.0f, 1.0f));

		this->_m3 = 0.0f;
		this->_m7 = 0.0f;
		this->_m11 = 0.0f;
		this->_m15 = 1.0f;

		switch (type)
		{
		case Orient::LocalToWorld:
			break;
		case Orient::WorldToLocal:
			Mat4::T();
			break;
		}

		// Set rotation hint
		this->privSetRotHint();
	}

	void Rot::set(const Rot1 type, const float angle)
	{
		switch (type)
		{
		case Rot1::X:
			this->_m0 = 1.0f;
			this->_m1 = 0.0f;
			this->_m2 = 0.0f;
			this->_m3 = 0.0f;
			this->_m4 = 0.0f;
			this->_m5 = Trig::cos(angle);
			this->_m6 = Trig::sin(angle);
			this->_m7 = 0.0f;
			this->_m8 = 0.0f;
			this->_m9 = -Trig::sin(angle);
			this->_m10 = Trig::cos(angle);
			this->_m11 = 0.0f;
			this->_m12 = 0.0f;
			this->_m13 = 0.0f;
			this->_m14 = 0.0f;
			this->_m15 = 1.0f;
			break;
		case Rot1::Y:
			this->_m0 = Trig::cos(angle);
			this->_m1 = 0.0f;
			this->_m2 = -Trig::sin(angle);
			this->_m3 = 0.0f;
			this->_m4 = 0.0f;
			this->_m5 = 1.0f;
			this->_m6 = 0.0f;
			this->_m7 = 0.0f;
			this->_m8 = Trig::sin(angle);
			this->_m9 = 0.0f;
			this->_m10 = Trig::cos(angle);
			this->_m11 = 0.0f;
			this->_m12 = 0.0f;
			this->_m13 = 0.0f;
			this->_m14 = 0.0f;
			this->_m15 = 1.0f;
			break;
		case Rot1::Z:
			this->_m0 = Trig::cos(angle);
			this->_m1 = Trig::sin(angle);
			this->_m2 = 0.0f;
			this->_m3 = 0.0f;
			this->_m4 = -Trig::sin(angle);
			this->_m5 = Trig::cos(angle);
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
		}

		// Set rotation hint
		this->privSetRotHint();
	}

	void Rot::set(const Rot3 mode, const float angle_0, const float angle_1, const float angle_2)
	{
		Rot X = Rot(Rot1::X, angle_0);
		Rot Y = Rot(Rot1::Y, angle_1);
		Rot Z = Rot(Rot1::Z, angle_2);

		switch (mode)
		{
		case Rot3::XYZ:
			Mat4::set(X * Y * Z);
			break;
		case Rot3::XZY:
			Mat4::set(X * Z * Y);
			break;
		case Rot3::YXZ:
			Mat4::set(Y * X * Z);
			break;
		case Rot3::YZX:
			Mat4::set(Y * Z * X);
			break;
		case Rot3::ZXY:
			Mat4::set(Z * X * Y);
			break;
		case Rot3::ZYX:
			Mat4::set(Z * Y * X);
			break;
		}

		// Set rotation hint
		this->privSetRotHint();
	}

	void Rot::set(const Axis mode, const Vec3& vAxis, const float angle_radians)
	{
		const float angle_a = 0.5f * angle_radians;
		float cos_a;
		float sin_a;

		Trig::cossin(cos_a, sin_a, angle_a);

		Vec3 qV;

		switch (mode)
		{
		case Axis::AxisAngle:
			qV = vAxis.getNorm();
			break;
		case Axis::UnitAxisAngle:
			qV = vAxis;
			break;
		}

		qV *= sin_a;

		Vec4 Q;
		Q[x] = qV[x];
		Q[y] = qV[y];
		Q[z] = qV[z];
		Q[w] = cos_a;

		// this function has been transposed
		float x2, y2, z2;
		float xx, xy, xz;
		float yy, yz, zz;
		float wx, wy, wz;

		// ADD test to make sure that quat is normalized

		x2 = Q[x] + Q[x];
		y2 = Q[y] + Q[y];
		z2 = Q[z] + Q[z];

		xx = Q[x] * x2;
		xy = Q[x] * y2;
		xz = Q[x] * z2;

		yy = Q[y] * y2;
		yz = Q[y] * z2;
		zz = Q[z] * z2;

		wx = Q[w] * x2;
		wy = Q[w] * y2;
		wz = Q[w] * z2;

		this->_m0 = 1.0f - (yy + zz);
		this->_m1 = xy + wz;
		this->_m2 = xz - wy;
		this->_m3 = 0.0f;

		this->_m4 = xy - wz;
		this->_m5 = 1.0f - (xx + zz);
		this->_m6 = yz + wx;
		this->_m7 = 0.0f;

		this->_m8 = xz + wy;
		this->_m9 = yz - wx;
		this->_m10 = 1.0f - (xx + yy);
		this->_m11 = 0.0f;

		this->_v3.set(0.0f, 0.0f, 0.0f, 1.0f);

		// Set rotation hint
		this->privSetRotHint();
	}

	void Rot::set(const Orient type, const Vec3& dof, const Vec3& up)
	{
		/* make sure the DOF and VUP are not parallel */
		assert(dof.dot(up) != 1.0f);

		/* rz = vect_dof */
		Vec3 rz = dof.getNorm();

		/* find rx */
		Vec3 rx = up.cross(rz);
		rx.norm();

		/* find ry */
		Vec3 ry = rz.cross(rx);
		ry.norm();

		//this->set(Vec4(rx, 0), Vec4(ry, 0), Vec4(rz, 0), Vec4(0.0f, 0.0f, 0.0f, 1.0f));
		Mat4::set(Row4::i0, Vec4(rx, 0));
		Mat4::set(Row4::i1, Vec4(ry, 0));
		Mat4::set(Row4::i2, Vec4(rz, 0));
		Mat4::set(Row4::i3, Vec4(0.0f, 0.0f, 0.0f, 1.0f));

		this->_m3 = 0.0f;
		this->_m7 = 0.0f;
		this->_m11 = 0.0f;
		this->_m15 = 1.0f;

		switch (type)
		{
		case Orient::LocalToWorld:	
			break;
		case Orient::WorldToLocal:
			Mat4::T();
			break;
		}

		// Set rotation hint
		this->privSetRotHint();
	}

}