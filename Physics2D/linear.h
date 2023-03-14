#ifndef MATH_LINEAR_H
#define MATH_LINEAR_H
#include "vector2.h"
#include "vector3.h"
#include "vector4.h"
#include "matrix2x2.h"
#include "matrix3x3.h"
#include "matrix4x4.h"

namespace Physics2D
{
	inline Vector2 operator*(const real& f, const Vector2& v)
	{
		return v * f;
	}
	inline Vector3 operator*(const real& f, const Vector3& v)
	{
		return v * f;
	}
}
#endif