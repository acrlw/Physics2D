#ifndef MATH_LINEAR_H
#define MATH_LINEAR_H
#include "../../math/linear/vector2.h"
#include "../../math/linear/vector3.h"
#include "../../math/linear/vector4.h"
#include "../../math/linear/matrix2x2.h"
#include "../../math/linear/matrix3x3.h"
#include "../../math/linear/matrix4x4.h"

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