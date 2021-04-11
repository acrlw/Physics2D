#ifndef PHYSICS2D_LINEAR_VECTOR3
#define PHYSICS2D_LINEAR_VECTOR3
#include "include/math/math.h"
#include "include/common/common.h"
namespace Physics2D
{
	struct Vector3
	{
        Vector3(const real& _x = 0.0f, const real& _y = 0.0f, const real& _z = 0.0f);
        Vector3(const Vector3& copy);
        Vector3& operator=(const Vector3& copy);
		Vector3(Vector3&& other) = default;

        Vector3 operator+(const Vector3& rhs)const;
        Vector3 operator-(const Vector3& other)const;
        Vector3 operator*(const real& factor)const;
        Vector3 operator*(const int& factor)const;
        Vector3 operator/(const real& factor)const;
        Vector3 operator/(const int& factor)const;

        Vector3& operator+=(const Vector3& rhs);
        Vector3& operator-=(const Vector3& rhs);
        Vector3& operator*=(const real& factor);
        Vector3& operator*=(const int& factor);
        Vector3& operator/=(const real& factor);
        Vector3& operator/=(const int& factor);

        Vector3& set(const real& _x, const real& _y, const real& _z);
        Vector3& set(const Vector3& other);
        Vector3& clear();
        Vector3& negate();

        real lengthSquare()const;
        real length()const;

        Vector3& normalize();
        Vector3 normal()const;

        bool equal(const Vector3& rhs)const;
        Vector3& swap(Vector3& other);

        real dot(const Vector3& rhs)const;
        Vector3& cross(const Vector3& rhs);

        static real dotProduct(const Vector3& lhs, const Vector3& rhs);
        static Vector3 crossProduct(const Vector3& lhs, const Vector3& rhs);

        real x;
		real y;
		real z;
	};
}
#endif
