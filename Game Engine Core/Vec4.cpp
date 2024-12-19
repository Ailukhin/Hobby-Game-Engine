#include "MathEngine.h"
#include <corecrt_math.h>
#include <cassert>
#include <stdio.h>

namespace EngineCore
{
    Vec4::Vec4()
        :_vx(0.0f), _vy(0.0f), _vz(0.0f), _vw(0.0f)
    {

    }

    Vec4::Vec4(const Vec4& inV)
    {
        _vx = inV.x();
        _vy = inV.y();
        _vz = inV.z();
        _vw = inV.w();
    }

    Vec4& Vec4::operator=(const Vec4& v)
    {
        _vx = v.x();
        _vy = v.y();
        _vz = v.z();
        _vw = v.w();

        return *this;
    }

    Vec4::~Vec4()
    {
        _vx = 0.0f;
        _vy = 0.0f;
        _vz = 0.0f;
        _vw = 0.0f;
    }

    Vec4::Vec4(const float in_x, const float in_y, const float in_z, const float in_w)
    {
        _vx = in_x;
        _vy = in_y;
        _vz = in_z;
        _vw = in_w;
    }

    Vec4::Vec4(const Vec3& v, const float w)
    {
        _vx = v._vx;
        _vy = v._vy;
        _vz = v._vz;
        _vw = w;
    }

    float& Vec4::operator[](const x_enum)
    {
        return _vx;
    }

    float& Vec4::operator[](const y_enum)
    {
        return _vy;
    }

    float& Vec4::operator[](const z_enum)
    {
        return _vz;
    }

    float& Vec4::operator[](const w_enum)
    {
        return _vw;
    }


    float Vec4::operator[](const x_enum) const
    {
        return _vx;
    }

    float Vec4::operator[](const y_enum) const
    {
        return _vy;
    }

    float Vec4::operator[](const z_enum) const
    {
        return _vz;
    }

    float Vec4::operator[](const w_enum) const
    {
        return _vw;
    }

    void Vec4::x(const float v)
    {
        _vx = v;
    }

    void Vec4::y(const float v)
    {
        _vy = v;
    }

    void Vec4::z(const float v)
    {
        _vz = v;
    }

    void Vec4::w(const float v)
    {
        _vw = v;
    }

    float Vec4::x() const
    {
        return _vx;
    }

    float Vec4::y() const
    {
        return _vy;
    }

    float Vec4::z() const
    {
        return _vz;
    }

    float Vec4::w() const
    {
        return _vw;
    }

    Vec4 Vec4::operator+(void) const
    {
        return Vec4(_vx, _vy, _vz, _vw);
    }

    Vec4 Vec4::operator+(const Vec4& inV) const
    {
        return Vec4(_vx + inV._vx, _vy + inV._vy, _vz + inV._vz, _vw + inV._vw);
    }

    Vec4& Vec4::operator+=(const Vec4& inV)
    {
        _vx += inV._vx;
        _vy += inV._vy;
        _vz += inV._vz;
        _vw += inV._vw;

        return *this;
    }

    Vec4 Vec4::operator-(const Vec4& inV) const
    {
        return Vec4(_vx - inV._vx, _vy - inV._vy, _vz - inV._vz, _vw - inV._vw);
    }

    Vec4& Vec4::operator-=(const Vec4& inV)
    {
        _vx -= inV._vx;
        _vy -= inV._vy;
        _vz -= inV._vz;
        _vw -= inV._vw;

        return *this;
    }

    Vec4 Vec4::operator-(void) const
    {
        return Vec4(-_vx, -_vy, -_vz, -_vw);
    }

    Vec4 Vec4::operator*(const float scale) const
    {
        return Vec4(_vx * scale, _vy * scale, _vz * scale, _vw * scale);
    }

    Vec4& Vec4::operator*=(const float scale)
    {
        _vx *= scale;
        _vy *= scale;
        _vz *= scale;
        _vw *= scale;

        return *this;
    }

    Vec4 operator*(const float scale, const Vec4& inV)
    {
        return Vec4(inV._vx * scale, inV._vy * scale, inV._vz * scale, inV._vw * scale);
    }

    Vec4 Vec4::operator*(const Mat4& m) const
    {
        float x = _vx * m._m0 + _vy * m._m4 + _vz * m._m8 + _vw * m._m12;
        float y = _vx * m._m1 + _vy * m._m5 + _vz * m._m9 + _vw * m._m13;
        float z = _vx * m._m2 + _vy * m._m6 + _vz * m._m10 + _vw * m._m14;
        float w = _vx * m._m3 + _vy * m._m7 + _vz * m._m11 + _vw * m._m15;

        return Vec4(x, y, z, w);
    }

    Vec4& Vec4::operator*=(const Mat4& m)
    {
        float x = _vx * m._m0 + _vy * m._m4 + _vz * m._m8 + _vw * m._m12;
        float y = _vx * m._m1 + _vy * m._m5 + _vz * m._m9 + _vw * m._m13;
        float z = _vx * m._m2 + _vy * m._m6 + _vz * m._m10 + _vw * m._m14;
        float w = _vx * m._m3 + _vy * m._m7 + _vz * m._m11 + _vw * m._m15;

        _vx = x;
        _vy = y;
        _vz = z;
        _vw = w;

        return *this;
    }

    Vec4& Vec4::norm(void)
    {
        float n1 = (_vx * _vx) + (_vy * _vy) + (_vz * _vz) + (_vw * _vw);
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
        _vw /= n2;

        return *this;
    }

    Vec4 Vec4::getNorm(void) const
    {
        float n1 = (_vx * _vx) + (_vy * _vy) + (_vz * _vz) + (_vw * _vw);
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
        float w = _vw / n2;

        return Vec4(x, y, z, w);
    }

    float Vec4::dot(const Vec4& vIn) const
    {
        return (_vx * vIn._vx) + (_vy * vIn._vy) + (_vz * vIn._vz) + (_vw * vIn._vw);
    }

    Vec4Proxy Vec4::len() const
    {
        /*float n1 = (_vx * _vx) + (_vy * _vy) + (_vz * _vz) + (_vw * _vw);
        float n2 = sqrtf(n1);*/

        Vec4Proxy prox = Vec4Proxy(_vx, _vy, _vz, _vw);

        return prox;
    }

    void Vec4::set(const float inX, const float inY, const float inZ, const float inW)
    {
        _vx = inX;
        _vy = inY;
        _vz = inZ;
        _vw = inW;
    }

    void Vec4::set(const Vec4& A)
    {
        _vx = A._vx;
        _vy = A._vy;
        _vz = A._vz;
        _vw = A._vw;
    }

    void Vec4::set(const Vec3& v, const float w)
    {
        _vx = v._vx;
        _vy = v._vy;
        _vz = v._vz;
        _vw = w;
    }

    bool Vec4::isEqual(const Vec4& v, const float epsilon) const
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
        else if (fabsf(v.w() - _vw) > epsilon)
        {
            ret = false;
        }

        return ret;
    }

    bool Vec4::isZero(const float epsilon) const
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
        else if (_vw > epsilon || _vw < -epsilon)
        {
            ret = false;
        }

        return ret;
    }
}