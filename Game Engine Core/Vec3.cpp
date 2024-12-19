#include "MathEngine.h"
#include <cassert>
#include <corecrt_math.h>
#include <stdio.h>

namespace EngineCore
{
    Vec3::Vec3()
        :_vx(0.0f), _vy(0.0f), _vz(0.0f)
    {

    }

    Vec3::Vec3(const Vec3& inV)
    {
        _vx = inV.x();
        _vy = inV.y();
        _vz = inV.z();
    }

    Vec3& Vec3::operator=(const Vec3& v)
    {
        _vx = v.x();
        _vy = v.y();
        _vz = v.z();

        return *this;
    }

    Vec3::~Vec3()
    {
        _vx = 0.0f;
        _vy = 0.0f;
        _vz = 0.0f;
    }

    Vec3::Vec3(const float in_x, const float in_y, const float in_z)
    {
        _vx = in_x;
        _vy = in_y;
        _vz = in_z;
    }

    Vec3::Vec3(const Vec4& v)
    {
        _vx = v.x();
        _vy = v.y();
        _vz = v.z();
    }

    Vec3& Vec3::operator=(const Vec4& v)
    {
        _vx = v.x();
        _vy = v.y();
        _vz = v.z();

        return *this;
    }

    float &Vec3::operator[](const x_enum)
    {
        return _vx;
    }

    float &Vec3::operator[](const y_enum)
    {
        return _vy;
    }

    float &Vec3::operator[](const z_enum)
    {
        return _vz;
    }

    float Vec3::operator[](const x_enum) const
    {
        return _vx;
    }

    float Vec3::operator[](const y_enum) const
    {
        return _vy;
    }

    float Vec3::operator[](const z_enum) const
    {
        return _vz;
    }

    void Vec3::x(const float v)
    {
        _vx = v;
    }

    void Vec3::y(const float v)
    {
        _vy = v;
    }

    void Vec3::z(const float v)
    {
        _vz = v;
    }

    float Vec3::x() const
    {
        return _vx;
    }

    float Vec3::y() const
    {
        return _vy;
    }

    float Vec3::z() const
    {
        return _vz;
    }

    Vec3 Vec3::operator+(void) const
    {
        return Vec3(_vx, _vy, _vz);
    }

    Vec3 Vec3::operator+(const Vec3& inV) const
    {
        return Vec3(_vx + inV._vx, _vy + inV._vy, _vz + inV._vz);
    }

    Vec3& Vec3::operator+=(const Vec3& inV)
    {
        _vx += inV._vx;
        _vy += inV._vy;
        _vz += inV._vz;

        return *this;
    }

    Vec3 Vec3::operator-(const Vec3& inV) const
    {
        return Vec3(_vx - inV.x(), _vy - inV.y(), _vz - inV.z());
    }

    Vec3& Vec3::operator-=(const Vec3& inV)
    {
        _vx -= inV._vx;
        _vy -= inV._vy;
        _vz -= inV._vz;

        return *this;
    }

    Vec3 Vec3::operator-(void) const
    {
        return Vec3(-_vx, -_vy, -_vz);
    }

    Vec3 Vec3::operator*(const float scale) const
    {
        return Vec3(_vx * scale, _vy * scale, _vz * scale);
    }

    Vec3& Vec3::operator*=(const float scale)
    {
        _vx *= scale;
        _vy *= scale;
        _vz *= scale;

        return *this;
    }

    Vec3 operator*(const float scale, const Vec3& inV)
    {
        return Vec3(inV._vx * scale, inV._vy * scale, inV._vz * scale);
    }

    Vec3 Vec3::operator*(const Mat3& m) const
    {
        float x = _vx * m._m0 + _vy * m._m4 + _vz * m._m8;
        float y = _vx * m._m1 + _vy * m._m5 + _vz * m._m9;
        float z = _vx * m._m2 + _vy * m._m6 + _vz * m._m10;

        return Vec3(x, y, z);
    }

    Vec3& Vec3::operator*=(const Mat3& m)
    {
        float x = _vx * m._m0 + _vy * m._m4 + _vz * m._m8;
        float y = _vx * m._m1 + _vy * m._m5 + _vz * m._m9;
        float z = _vx * m._m2 + _vy * m._m6 + _vz * m._m10;

        _vx = x;
        _vy = y;
        _vz = z;

        return *this;
    }

    Vec3 Vec3::operator*(const Quat& q) const
    {
        return Vec3(*this) * Mat3(Mat4(q));
    }

    Vec3& Vec3::operator*=(const Quat& q)
    {
        *this = Vec3(*this) * Mat3(Mat4(q));

        return *this;
    }

    Vec3& Vec3::norm(void)
    {
        float n1 = (_vx * _vx) + (_vy * _vy) + (_vz * _vz);
        float n2 = sqrtf(n1);

        // Can't divide by 0, assert
        if (n2 == 0)
        {
            printf("Tried to normalize a (0, 0, 0) vector\n");
            assert(false);
        }

        _vx /= n2;
        _vy /= n2;
        _vz /= n2;

        return *this;
    }

    Vec3 Vec3::getNorm(void) const
    {
        float n1 = (_vx * _vx) + (_vy * _vy) + (_vz * _vz);
        float n2 = sqrtf(n1);

        // Can't divide by 0, assert
        if (n2 == 0)
        {
            printf("Tried to normalize a (0, 0, 0) vector\n");
            assert(false);
        }

        float x = _vx / n2;
        float y = _vy / n2;
        float z = _vz / n2;

        return Vec3(x, y, z);
    }

    float Vec3::dot(const Vec3& vIn) const
    {
        return (_vx * vIn._vx) + (_vy * vIn._vy) + (_vz * vIn._vz);
    }

    Vec3 Vec3::cross(const Vec3& vIn) const
    {
        float crossX = _vy * vIn._vz - _vz * vIn._vy;
        float crossY = -(_vx * vIn._vz - _vz * vIn._vx);
        float crossZ = _vx * vIn._vy - _vy * vIn._vx;

        return Vec3(crossX, crossY, crossZ);
    }

    Vec3Proxy Vec3::len(void) const
    {
        /*float n1 = (_vx * _vx) + (_vy * _vy) + (_vz * _vz);
        float n2 = sqrtf(n1);*/

        Vec3Proxy prox = Vec3Proxy(_vx, _vy, _vz);

        return prox;
    }

    float Vec3::getAngle(const Vec3& vIn) const
    {
        float dt = this->dot(vIn);
        float mag = this->len() * vIn.len();

        float ret = Trig::acos(dt/mag);

        return ret;
    }

    void Vec3::set(const float inX, const float inY, const float inZ)
    {
        _vx = inX;
        _vy = inY;
        _vz = inZ;
    }

    void Vec3::set(const Vec3& A)
    {
        _vx = A.x();
        _vy = A.y();
        _vz = A.z();
    }

    void Vec3::set(const Vec4& A)
    {
        _vx = A.x();
        _vy = A.y();
        _vz = A.z();
    }

    bool Vec3::isEqual(const Vec3& v, const float epsilon) const
    {
        bool ret = true;

        if (fabsf(v.x() - _vx) > epsilon)
        {
            ret = false;
        }
        else if (fabsf(v.y() - _vy) > epsilon)
        {
            ret = false;
        }
        else if (fabsf(v.z() - _vz) > epsilon)
        {
            ret = false;
        }

        return ret;
    }

    bool Vec3::isZero(const float epsilon) const
    {
        bool ret = true;

        if (_vx > epsilon || _vx < -epsilon)
        {
            ret = false;
        }
        else if (_vy > epsilon || _vy < -epsilon)
        {
            ret = false;
        }
        else if (_vz > epsilon || _vz < -epsilon)
        {
            ret = false;
        }

        return ret;
    }

    /*void Vec3::Print(const char* pName) const
    {

    }*/

}