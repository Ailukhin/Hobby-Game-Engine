#include "MathEngine.h"
#include <corecrt_math.h>
#include <cassert>
#include <cstdlib>

namespace EngineCore
{ 
	Quat::Quat(void)
	{
		this->_qx = 0.0f;
		this->_qy = 0.0f;
		this->_qz = 0.0f;
		this->_qw = 1.0f;
	}

	Quat::Quat(const Quat& qIn)
	{
		this->_qx = qIn._qx;
		this->_qy = qIn._qy;
		this->_qz = qIn._qz;
		this->_qw = qIn._qw;
	}

	Quat::Quat(const Vec3& vect, const float real)
	{
		this->_qV3 = vect;
		this->_qw = real;
	}

	Quat::Quat(const float vx, const float vy, const float vz, const float real)
		:_qx(vx), _qy(vy), _qz(vz), _qw(real)
	{

	}

	Quat::Quat(const Rot& mIn)
	{
		float tr = mIn._m0 + mIn._m5 + mIn._m10;
		float S;

		if (tr > 0.0f)
		{
			S = 2 * Trig::sqrt(tr + 1.0f);
			this->_qw = 0.25f * S;
			this->_qx = -(mIn._m9 - mIn._m6) / S;
			this->_qy = -(mIn._m2 - mIn._m8) / S;
			this->_qz = -(mIn._m4 - mIn._m1) / S;
		}
		else if ((mIn._m0 > mIn._m5) && (mIn._m0 > mIn._m10))
		{
			S = 2 * Trig::sqrt(1.0f + mIn._m0 - mIn._m5 - mIn._m10);
			this->_qw = -(mIn._m9 - mIn._m6) / S;
			this->_qx = 0.25f * S;
			this->_qy = (mIn._m1 + mIn._m4) / S;
			this->_qz = (mIn._m2 + mIn._m8) / S;
		}
		else if (mIn._m5 > mIn._m10)
		{
			S = 2 * Trig::sqrt(1.0f + mIn._m5 - mIn._m0 - mIn._m10);
			this->_qw = -(mIn._m2 - mIn._m8) / S;
			this->_qx = (mIn._m1 + mIn._m4) / S;
			this->_qy = 0.25f * S;
			this->_qz = (mIn._m6 + mIn._m9) / S;
		}
		else
		{
			S = 2 * Trig::sqrt(1.0f + mIn._m10 - mIn._m0 - mIn._m5);
			this->_qw = -(mIn._m4 - mIn._m1) / S;
			this->_qx = (mIn._m2 + mIn._m8) / S;
			this->_qy = (mIn._m6 + mIn._m9) / S;
			this->_qz = 0.25f * S;
		}
	}

	Quat::Quat(const Special value)
	{
		switch (value)
		{
		case Special::Identity:
			this->_qx = 0.0f;
			this->_qy = 0.0f;
			this->_qz = 0.0f;
			this->_qw = 1.0f;
			break;
		case Special::Zero:
			this->_qx = 0.0f;
			this->_qy = 0.0f;
			this->_qz = 0.0f;
			this->_qw = 0.0f;
			break;
		}
	}

	Quat::Quat(const Rot1 type, const float angle)
	{
		float tr;
		float S;

		switch (type)
		{
		case Rot1::X:

			tr = 1.0f + Trig::cos(angle) + Trig::cos(angle);

			if (tr > 0.0f)
			{
				S = 2 * Trig::sqrt(1.0f + tr);
				this->_qx = -(-Trig::sin(angle) - Trig::sin(angle)) / S;
				this->_qy = 0.0f;
				this->_qz = 0.0f;
				this->_qw = 0.25f * S;
			}
			else if (1.0f > Trig::cos(angle))
			{
				S = 2 * Trig::sqrt(1.0f + 1.0f - Trig::cos(angle) - Trig::cos(angle));
				this->_qx = 0.25f * S;
				this->_qy = 0.0f;
				this->_qz = 0.0f;
				this->_qw = -(-Trig::sin(angle) - Trig::sin(angle)) / S;
			}
			else
			{
				assert(false);
				// Rot::X case not accounted for
			}

			break;
		case Rot1::Y:

			tr = Trig::cos(angle) + 1.0f + Trig::cos(angle);

			if (tr > 0.0f)
			{
				S = 2 * Trig::sqrt(1.0f + tr);
				this->_qx = 0.0f;
				this->_qy = -(-Trig::sin(angle) - Trig::sin(angle)) / S;
				this->_qz = 0.0f;
				this->_qw = 0.25f * S;
			}
			else if (1.0f > Trig::cos(angle))
			{
				S = 2 * Trig::sqrt(1.0f + 1.0f - Trig::cos(angle) - Trig::cos(angle));
				this->_qx = 0.0f;
				this->_qy = 0.25f * S;
				this->_qz = 0.0f;
				this->_qw = -(-Trig::sin(angle) - Trig::sin(angle)) / S;;
			}
			else
			{
				assert(false);
				// Rot::Y case not accounted for
			}

			break;
		case Rot1::Z:

			tr = Trig::cos(angle) + Trig::cos(angle) + 1.0f;

			if (tr > 0.0f)
			{
				S = 2 * Trig::sqrt(1.0f + tr);
				this->_qx = 0.0f;
				this->_qy = 0.0f;
				this->_qz = -(-Trig::sin(angle) - Trig::sin(angle)) / S;
				this->_qw = 0.25f * S;
			}
			else if (Trig::cos(angle) > 1.0f) // I don't think this will be hit...?
			{
				S = 2 * Trig::sqrt(1.0f + Trig::cos(angle) - Trig::cos(angle) + 1.0f);
				this->_qx = (Trig::sin(angle) - Trig::sin(angle)) / S;;
				this->_qy = 0.25f * S;
				this->_qz = 0.0f;
				this->_qw = 0.0f;
			}
			else
			{
				S = 2 * Trig::sqrt(1.0f + 1.0f - Trig::cos(angle) - Trig::cos(angle));
				this->_qx = 0.0f;
				this->_qy = 0.0f;
				this->_qz = 0.25f * S;
				this->_qw = -(-Trig::sin(angle) - Trig::sin(angle)) / S;
			}

			break;
		}
	}

	Quat::Quat(const Rot3 type, const float angle_x, const float angle_y, const float angle_z)
	{
		Quat X = Quat(Rot1::X, angle_x);
		Quat Y = Quat(Rot1::Y, angle_y);
		Quat Z = Quat(Rot1::Z, angle_z);

		Quat combo;

		switch (type)
		{
		case Rot3::XYZ:
			combo = X * Y * Z;
			break;
		case Rot3::XZY:
			combo = X * Z * Y;
			break;
		case Rot3::YXZ:
			combo = Y * X * Z;
			break;
		case Rot3::YZX:
			combo = Y * Z * X;
			break;
		case Rot3::ZXY:
			combo = Z * X * Y;
			break;
		case Rot3::ZYX:
			combo = Z * Y * X;
			break;
		}

		*this = combo;
	}

	Quat::Quat(const Axis type, const Vec3& vAxis, const float angle_radians)
	{
		const float angle_a = 0.5f * angle_radians;
		float cos_a;
		float sin_a;

		Trig::cossin(cos_a, sin_a, angle_a);

		Vec3 qV;

		switch (type)
		{
		case Axis::AxisAngle:
			qV = vAxis.getNorm();
			break;
		case Axis::UnitAxisAngle:
			qV = vAxis;
			break;
		}

		qV *= sin_a;

		this->_qx = qV[x];
		this->_qy = qV[y];
		this->_qz = qV[z];
		this->_qw = cos_a;
	}

	Quat::Quat(const Orient type, const Vec3& dof, const Vec3& up)
	{
		Rot orient = Rot(type, dof, up);

		*this = Quat(orient);
	}

	Quat::~Quat()
	{

	}

	void Quat::set(const Special type)
	{
		switch (type)
		{
		case Special::Identity:
			this->_qx = 0.0f;
			this->_qy = 0.0f;
			this->_qz = 0.0f;
			this->_qw = 1.0f;
			break;
		case Special::Zero:
			this->_qx = 0.0f;
			this->_qy = 0.0f;
			this->_qz = 0.0f;
			this->_qw = 0.0f;
			break;
		}
	}

	void Quat::set(const Rot1 type, const float angle)
	{
		float tr;
		float S;

		switch (type)
		{
		case Rot1::X:

			tr = 1.0f + Trig::cos(angle) + Trig::cos(angle);

			if (tr > 0.0f)
			{
				S = 2 * Trig::sqrt(1.0f + tr);
				this->_qx = -(-Trig::sin(angle) - Trig::sin(angle)) / S;
				this->_qy = 0.0f;
				this->_qz = 0.0f;
				this->_qw = 0.25f * S;
			}
			else if (1.0f > Trig::cos(angle))
			{
				S = 2 * Trig::sqrt(1.0f + 1.0f - Trig::cos(angle) - Trig::cos(angle));
				this->_qx = 0.25f * S;
				this->_qy = 0.0f;
				this->_qz = 0.0f;
				this->_qw = -(-Trig::sin(angle) - Trig::sin(angle)) / S;
			}
			else
			{
				assert(false);
				// Rot::X case not accounted for
			}

			break;
		case Rot1::Y:

			tr = Trig::cos(angle) + 1.0f + Trig::cos(angle);

			if (tr > 0.0f)
			{
				S = 2 * Trig::sqrt(1.0f + tr);
				this->_qx = 0.0f;
				this->_qy = -(-Trig::sin(angle) - Trig::sin(angle)) / S;
				this->_qz = 0.0f;
				this->_qw = 0.25f * S;
			}
			else if (1.0f > Trig::cos(angle))
			{
				S = 2 * Trig::sqrt(1.0f + 1.0f - Trig::cos(angle) - Trig::cos(angle));
				this->_qx = 0.0f;
				this->_qy = 0.25f * S;
				this->_qz = 0.0f;
				this->_qw = -(-Trig::sin(angle) - Trig::sin(angle)) / S;;
			}
			else
			{
				assert(false);
				// Rot::Y case not accounted for
			}

			break;
		case Rot1::Z:

			tr = Trig::cos(angle) + Trig::cos(angle) + 1.0f;

			if (tr > 0.0f)
			{
				S = 2 * Trig::sqrt(1.0f + tr);
				this->_qx = 0.0f;
				this->_qy = 0.0f;
				this->_qz = -(-Trig::sin(angle) - Trig::sin(angle)) / S;
				this->_qw = 0.25f * S;
			}
			else if (Trig::cos(angle) > 1.0f) // I don't think this will be hit...?
			{
				S = 2 * Trig::sqrt(1.0f + Trig::cos(angle) - Trig::cos(angle) + 1.0f);
				this->_qx = (Trig::sin(angle) - Trig::sin(angle)) / S;;
				this->_qy = 0.25f * S;
				this->_qz = 0.0f;
				this->_qw = 0.0f;
			}
			else
			{
				S = 2 * Trig::sqrt(1.0f + 1.0f - Trig::cos(angle) - Trig::cos(angle));
				this->_qx = 0.0f;
				this->_qy = 0.0f;
				this->_qz = 0.25f * S;
				this->_qw = -(-Trig::sin(angle) - Trig::sin(angle)) / S;
			}

			break;
		}
	}

	void Quat::set(const Rot3 type, const float angle_x, const float angle_y, const float angle_z)
	{
		Quat X = Quat(Rot1::X, angle_x);
		Quat Y = Quat(Rot1::Y, angle_y);
		Quat Z = Quat(Rot1::Z, angle_z);

		Quat combo;

		switch (type)
		{
		case Rot3::XYZ:
			combo = X * Y * Z;
			break;
		case Rot3::XZY:
			combo = X * Z * Y;
			break;
		case Rot3::YXZ:
			combo = Y * X * Z;
			break;
		case Rot3::YZX:
			combo = Y * Z * X;
			break;
		case Rot3::ZXY:
			combo = Z * X * Y;
			break;
		case Rot3::ZYX:
			combo = Z * Y * X;
			break;
		}

		*this = combo;
	}

	void Quat::set(const Axis type, const Vec3& vAxis, const float angle_radians)
	{
		const float angle_a = 0.5f * angle_radians;
		float cos_a;
		float sin_a;

		Trig::cossin(cos_a, sin_a, angle_a);

		Vec3 qV;

		switch (type)
		{
		case Axis::AxisAngle:
			qV = vAxis.getNorm();
			break;
		case Axis::UnitAxisAngle:
			qV = vAxis;
			break;
		}

		qV *= sin_a;

		this->_qx = qV[x];
		this->_qy = qV[y];
		this->_qz = qV[z];
		this->_qw = cos_a;
	}

	void Quat::set(const Orient type, const Vec3& dof, const Vec3& up)
	{
		Rot orient = Rot(type, dof, up);

		*this = Quat(orient);
	}

	void Quat::set(const float vx, const float vy, const float vz, const float real)
	{
		this->_qx = vx;
		this->_qy = vy;
		this->_qz = vz;
		this->_qw = real;
	}

	float& Quat::operator[](const x_enum)
	{
		return this->_qx;
	}

	float& Quat::operator[](const y_enum)
	{
		return this->_qy;
	}

	float& Quat::operator[](const z_enum)
	{
		return this->_qz;
	}

	float& Quat::operator[](const w_enum)
	{
		return this->_qw;
	}

	void Quat::qx(const float f)
	{
		this->_qx = f;
	}

	void Quat::qy(const float f)
	{
		this->_qy = f;
	}

	void Quat::qz(const float f)
	{
		this->_qz = f;
	}

	void Quat::real(const float f)
	{
		this->_qw = f;
	}

	void Quat::setVec3(const Vec3& vect)
	{
		this->_qV3 = vect;
		this->_qw = 1.0f;
	}

	void Quat::set(const Vec3& vect, const float real)
	{
		this->_qV3 = vect;
		this->_qw = real;
	}

	void Quat::set(const Rot& m)
	{
		float tr = m._m0 + m._m5 + m._m10;
		float S;

		if (tr > 0.0f)
		{
			S = 2 * Trig::sqrt(tr + 1.0f);
			this->_qw = 0.25f * S;
			this->_qx = -(m._m9 - m._m6) / S;
			this->_qy = -(m._m2 - m._m8) / S;
			this->_qz = -(m._m4 - m._m1) / S;
		}
		else if ((m._m0 > m._m5) && (m._m0 > m._m10))
		{
			S = 2 * Trig::sqrt(1.0f + m._m0 - m._m5 - m._m10);
			this->_qw = -(m._m9 - m._m6) / S;
			this->_qx = 0.25f * S;
			this->_qy = (m._m1 + m._m4) / S;
			this->_qz = (m._m2 + m._m8) / S;
		}
		else if (m._m5 > m._m10)
		{
			S = 2 * Trig::sqrt(1.0f + m._m5 - m._m0 - m._m10);
			this->_qw = -(m._m2 - m._m8) / S;
			this->_qx = (m._m1 + m._m4) / S;
			this->_qy = 0.25f * S;
			this->_qz = (m._m6 + m._m9) / S;
		}
		else
		{
			S = 2 * Trig::sqrt(1.0f + m._m10 - m._m0 - m._m5);
			this->_qw = -(m._m4 - m._m1) / S;
			this->_qx = (m._m2 + m._m8) / S;
			this->_qy = (m._m6 + m._m9) / S;
			this->_qz = 0.25f * S;
		}
	}

	float Quat::operator[](const x_enum) const
	{
		return this->_qx;
	}

	float Quat::operator[](const y_enum) const
	{
		return this->_qy;
	}

	float Quat::operator[](const z_enum) const
	{
		return this->_qz;
	}

	float Quat::operator[](const w_enum) const
	{
		return this->_qw;
	}

	float Quat::qx() const
	{
		return this->_qx;
	}

	float Quat::qy() const
	{
		return this->_qy;
	}

	float Quat::qz() const
	{
		return this->_qz;
	}

	float Quat::real() const
	{
		return this->_qw;
	}

	float Quat::getAngle(void) const
	{
		return Trig::acos(this->_qw) / 0.5f;
	}

	void Quat::getVec3(Vec3& vOut) const
	{
		vOut._vx = this->_qx;
		vOut._vy = this->_qy;
		vOut._vz = this->_qz;
	}

	void Quat::getAxis(Vec3& vOut) const
	{
		/*float originalAngle = Trig::acos(this->_qw) / 0.5f;
		
		float s = Trig::sin(originalAngle);*/

		vOut = Vec3(this->_qx * 2.0f, this->_qy * 2.0f, this->_qz * 2.0f);
	}

	float Quat::dot(const Quat& qin) const
	{
		return this->_qx * qin._qx + this->_qy * qin._qy + this->_qz * qin._qz + this->_qw * qin._qw;
	}

	float Quat::mag(void) const
	{
		return Trig::sqrt(this->_qx * this->_qx + this->_qy * this->_qy + this->_qz * this->_qz + this->_qw * this->_qw);
	}

	float Quat::magSquared(void) const
	{
		return this->_qx * this->_qx + this->_qy * this->_qy + this->_qz * this->_qz + this->_qw * this->_qw;
	}

	float Quat::invMag(void) const
	{
		return 1.0f / this->mag();
	}

	Quat& Quat::conj(void)
	{
		this->_qx = -this->_qx;
		this->_qy = -this->_qy;
		this->_qz = -this->_qz;

		return *this;
	}

	Quat Quat::getConj(void) const
	{
		return Quat(-this->_qx, -this->_qy, -this->_qz, this->_qw);
	}

	Quat& Quat::inv(void)
	{
		this->conj();

		return *this;
	}

	Quat Quat::getInv(void) const
	{
		return this->getConj();
	}

	Quat& Quat::norm(void)
	{
		float sq = this->mag();

		this->_qx = this->_qx / sq;
		this->_qy = this->_qy / sq;
		this->_qz = this->_qz / sq;
		this->_qw = this->_qw / sq;

		return *this;
	}

	Quat Quat::getNorm(void) const
	{
		float sq = this->mag();

		return Quat(this->_qx / sq, this->_qy / sq, this->_qz / sq, this->_qw / sq);
	}

	void Quat::Lqvqc(const Vec3& vIn, Vec3& vOut) const
	{
		// vOut = Quat * vIn * Quat.conj()

		vOut = (2.0f * this->_qw * vIn.cross(this->_qV3)) + (vIn * (this->_qw * this->_qw - this->_qV3.dot(this->_qV3))) + (2 * (this->_qV3.dot(vIn)) * this->_qV3);
	}

	void Quat::Lqcvq(const Vec3& vIn, Vec3& vOut) const
	{
		// vOut = Quat.conj() * vIn * Quat	
	
		vOut = (2.0f * this->_qw * this->_qV3.cross(vIn)) + (vIn * (this->_qw * this->_qw - this->_qV3.dot(this->_qV3))) + (2 * (this->_qV3.dot(vIn)) * this->_qV3);
	}

	Mat4 Quat::operator*(const Mat4& m) const
	{
		return Mat4(*this) * m;
	}

	Quat Quat::operator*(const Quat& q) const
	{
		Vec3 vec = q._qV3.cross(this->_qV3) + q._qw * this->_qV3 + this->_qw * q._qV3;
		float w = this->_qw * q._qw - this->_qV3.dot(q._qV3);

		return Quat(vec, w);
	}

	Quat& Quat::operator*=(const Quat& q)
	{
		Vec3 vec = q._qV3.cross(this->_qV3) + q._qw * this->_qV3 + this->_qw * q._qV3;
		float w = this->_qw * q._qw - this->_qV3.dot(q._qV3);

		this->_qx = vec.x();
		this->_qy = vec.y();
		this->_qz = vec.z();
		this->_qw = w;

		return *this;
	}

	Mat4 Quat::operator*(const Scale& m) const
	{
		return Mat4(*this) * m;
	}

	Quat& Quat::operator*=(const Rot& m)
	{
		Rot x = Rot(*this) * m;

		*this = x;

		return *this;
	}

	Rot Quat::operator*(const Rot& m) const
	{
		return Rot(*this) * m;
	}

	Mat4 Quat::operator*(const Trans& m) const
	{
		return Mat4(*this) * m;
	}

	Quat Quat::operator*(const float a) const
	{
		return Quat(this->_qx * a, this->_qy * a, this->_qz * a, this->_qw * a);
	}

	Quat& Quat::operator*=(const float a)
	{
		this->_qx = this->_qx * a;
		this->_qy = this->_qy * a;
		this->_qz = this->_qz * a;
		this->_qw = this->_qw * a;

		return *this;
	}

	/*Quat& Quat::norm(void)
	{
		

		return *this;
	}

	Quat Quat::getNorm(void) const
	{
		return Quat();
	}*/

	bool Quat::isEqual(const Quat& qin, const float epsilon) const
	{
		bool ret = true;

		if (fabsf(qin._qx - this->_qx) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(qin._qy - this->_qy) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(qin._qz - this->_qz) > epsilon)
		{
			ret = false;
		}
		else if (fabsf(qin._qw - this->_qw) > epsilon)
		{
			ret = false;
		}

		return ret;
	}

	bool Quat::isEquivalent(const Quat& qin, const float epsilon) const
	{
		bool ret = true;

		if ((qin._qx > 0.0f && qin._qy > 0.0f && qin._qz > 0.0f && qin._qw > 0.0f) || (qin._qx < 0.0f && qin._qy < 0.0f && qin._qz < 0.0f && qin._qw < 0.0f))
		{
			if ((this->_qx > 0.0f && this->_qy > 0.0f && this->_qz > 0.0f && this->_qw > 0.0f) || (this->_qx < 0.0f && this->_qy < 0.0f && this->_qz < 0.0f && this->_qw < 0.0f))
			{
				if ((fabsf(qin._qx) - fabsf(this->_qx)) > epsilon)
				{
					ret = false;
				}
				else if ((fabsf(qin._qy) - fabsf(this->_qy)) > epsilon)
				{
					ret = false;
				}
				else if ((fabsf(qin._qz) - fabsf(this->_qz)) > epsilon)
				{
					ret = false;
				}
				else if ((fabsf(qin._qw) - fabsf(this->_qw)) > epsilon)
				{
					ret = false;
				}
			}
		}
		else
		{
			ret = false;
		}

		return ret;
	}

	bool Quat::isNegEqual(const Quat& qin, const float epsilon) const
	{
		bool ret = true;

		if ((fabsf(qin._qx) - fabsf(this->_qx)) > epsilon)
		{
			ret = false;
		}
		else if ((fabsf(qin._qy) - fabsf(this->_qy)) > epsilon)
		{
			ret = false;
		}
		else if ((fabsf(qin._qz) - fabsf(this->_qz)) > epsilon)
		{
			ret = false;
		}
		else if ((fabsf(qin._qw) - fabsf(this->_qw)) > epsilon)
		{
			ret = false;
		}

		return ret;
	}

	bool Quat::isConjugateEqual(const Quat& qin, const float epsilon) const
	{
		Quat conj = qin.getConj();

		return this->isEqual(conj, epsilon);
	}

	bool Quat::isIdentity(const float epsilon) const
	{
		bool ret = true;

		if (this->_qx > epsilon || this->_qx < -epsilon)
		{
			ret = false;
		}
		else if (this->_qy > epsilon || this->_qy < -epsilon)
		{
			ret = false;
		}
		else if (this->_qz > epsilon || this->_qz < -epsilon)
		{
			ret = false;
		}
		else if (this->_qw > (1.0f + epsilon) || this->_qw < (-1.0f - epsilon))
		{
			ret = false;
		}

		return ret;
	}

	bool Quat::isNormalized(const float epsilon) const
	{
		float x = this->magSquared();

		return x < (1.0f + epsilon);
	}

	bool Quat::isZero(const float epsilon) const
	{
		bool ret = true;

		if (this->_qx > epsilon || this->_qx < -epsilon)
		{
			ret = false;
		}
		else if (this->_qy > epsilon || this->_qy < -epsilon)
		{
			ret = false;
		}
		else if (this->_qz > epsilon || this->_qz < -epsilon)
		{
			ret = false;
		}
		else if (this->_qw > epsilon || this->_qw < -epsilon)
		{
			ret = false;
		}

		return ret;
	}

	Quat& Quat::operator=(const Quat& q)
	{
		this->_qx = q._qx;
		this->_qy = q._qy;
		this->_qz = q._qz;
		this->_qw = q._qw;

		return *this;
	}

	Quat& Quat::operator=(const Rot& m)
	{
		float tr = m._m0 + m._m5 + m._m10;
		float S;

		if (tr > 0.0f)
		{
			S = 2 * Trig::sqrt(tr + 1.0f);
			this->_qw = 0.25f * S;
			this->_qx = -(m._m9 - m._m6) / S;
			this->_qy = -(m._m2 - m._m8) / S;
			this->_qz = -(m._m4 - m._m1) / S;
		}
		else if ((m._m0 > m._m5) && (m._m0 > m._m10))
		{
			S = 2 * Trig::sqrt(1.0f + m._m0 - m._m5 - m._m10);
			this->_qw = -(m._m9 - m._m6) / S;
			this->_qx = 0.25f * S;
			this->_qy = (m._m1 + m._m4) / S;
			this->_qz = (m._m2 + m._m8) / S;
		}
		else if (m._m5 > m._m10)
		{
			S = 2 * Trig::sqrt(1.0f + m._m5 - m._m0 - m._m10);
			this->_qw = -(m._m2 - m._m8) / S;
			this->_qx = (m._m1 + m._m4) / S;
			this->_qy = 0.25f * S;
			this->_qz = (m._m6 + m._m9) / S;
		}
		else
		{
			S = 2 * Trig::sqrt(1.0f + m._m10 - m._m0 - m._m5);
			this->_qw = -(m._m4 - m._m1) / S;
			this->_qx = (m._m2 + m._m8) / S;
			this->_qy = (m._m6 + m._m9) / S;
			this->_qz = 0.25f * S;
		}

		return *this;
	}

	Quat Quat::operator+(void) const
	{
		return Quat(fabsf(this->_qx), fabsf(this->_qy), fabsf(this->_qz), fabsf(this->_qw));
	}

	Quat Quat::operator+(const Quat& q) const
	{
		return Quat(this->_qx + q._qx, this->_qy + q._qy, this->_qz + q._qz, this->_qw + q._qw);
	}

	Quat& Quat::operator+=(const Quat& q)
	{
		this->_qx = this->_qx + q._qx;
		this->_qy = this->_qy + q._qy;
		this->_qz = this->_qz + q._qz;
		this->_qw = this->_qw + q._qw;

		return *this;
	}

	Quat Quat::operator+(const float a) const
	{
		return Quat(this->_qx + a, this->_qy + a, this->_qz + a, this->_qw + a);
	}

	Quat& Quat::operator+=(const float a)
	{
		this->_qx = this->_qx + a;
		this->_qy = this->_qy + a;
		this->_qz = this->_qz + a;
		this->_qw = this->_qw + a;

		return *this;
	}

	Quat Quat::operator-(void) const
	{
		return Quat(-this->_qx, -this->_qy, -this->_qz, -this->_qw);
	}

	Quat Quat::operator-(const Quat& q) const
	{
		return Quat(this->_qx - q._qx, this->_qy - q._qy, this->_qz - q._qz, this->_qw - q._qw);
	}

	Quat& Quat::operator-=(const Quat& q)
	{
		this->_qx = this->_qx - q._qx;
		this->_qy = this->_qy - q._qy;
		this->_qz = this->_qz - q._qz;
		this->_qw = this->_qw - q._qw;

		return *this;
	}

	Quat Quat::operator-(const float a) const
	{
		return Quat(this->_qx - a, this->_qy - a, this->_qz - a, this->_qw - a);
	}

	Quat& Quat::operator-=(const float a)
	{
		this->_qx = this->_qx - a;
		this->_qy = this->_qy - a;
		this->_qz = this->_qz - a;
		this->_qw = this->_qw - a;

		return *this;
	}

	Quat Quat::operator/(const Quat& q) const
	{
		return Quat(this->_qx / q._qx, this->_qy / q._qy, this->_qz / q._qz, this->_qw / q._qw);
	}

	Quat& Quat::operator/=(const Quat& q)
	{
		this->_qx = this->_qx / q._qx;
		this->_qy = this->_qy / q._qy;
		this->_qz = this->_qz / q._qz;
		this->_qw = this->_qw / q._qw;

		return *this;
	}

	Quat Quat::operator/(const float a) const
	{
		return Quat(this->_qx / a, this->_qy / a, this->_qz / a, this->_qw / a);
	}

	Quat& Quat::operator/=(const float a)
	{
		this->_qx = this->_qx / a;
		this->_qy = this->_qy / a;
		this->_qz = this->_qz / a;
		this->_qw = this->_qw / a;

		return *this;
	}

	Quat operator+(const float a, const Quat& q)
	{
		return Quat(q._qx + a, q._qy + a, q._qz + a, q._qw + a);
	}

	Quat operator-(const float a, const Quat& q)
	{
		return Quat(a - q._qx, a - q._qy, a - q._qz, a - q._qw);
	}

	Quat operator/(const float a, const Quat& q)
	{
		return Quat(a / q._qx, a / q._qy, a / q._qz, a / q._qw);
	}

	Quat operator*(const float a, const Quat& q)
	{
		return Quat(q._qx * a, q._qy * a, q._qz * a, q._qw * a);
	}

}