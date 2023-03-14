#ifndef MATH_LINEAR_VECTOR4_H
#define MATH_LINEAR_VECTOR4_H
#include "vector3.h"
namespace Physics2D
{
	struct Vector4
	{
		Vector4(const real& x = 0.0, const real& y = 0.0, const real& z = 0.0, const real& w = 0.0);
		Vector4& operator=(const Vector4& copy);
		Vector4& operator=(const Vector3& copy);
		Vector4(const Vector4& copy) = default;
		Vector4(Vector4&& other) = default;
		Vector4(const Vector3& copy);

        Vector4 operator+(const Vector4& rhs)const;
        Vector4 operator-(const Vector4& other)const;
        Vector4 operator-()const;
        Vector4 operator*(const real& factor)const;
        Vector4 operator/(const real& factor)const;

        Vector4& operator+=(const Vector4& rhs);
        Vector4& operator-=(const Vector4& rhs);
        Vector4& operator*=(const real& factor);
        Vector4& operator/=(const real& factor);

        Vector4& set(const real& x, const real& y, const real& z, const real& w);
        Vector4& set(const Vector4& other);
        Vector4& set(const Vector3& other);
        Vector4& clear();
        Vector4& negate();
        Vector4& normalize();

        real lengthSquare()const;
        real length()const;

        Vector4 normal()const;
        Vector4 negative()const;

        bool equal(const Vector4& rhs)const;
        bool fuzzyEqual(const Vector4& rhs, const real& epsilon = Constant::GeometryEpsilon)const;
        bool isOrigin(const real& epsilon = Constant::GeometryEpsilon)const;
        Vector4& swap(Vector4& other);

        real dot(const Vector4& rhs)const;
        Vector4& cross(const Vector4& rhs);

        static real dotProduct(const Vector4& lhs, const Vector4& rhs);
        static Vector4 crossProduct(const Vector4& lhs, const Vector4& rhs);


		real x;
		real y;
		real z;
		real w;
	};

	
}
#endif