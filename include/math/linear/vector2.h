#ifndef PHYSICS2D_LINEAR_VECTOR2
#define PHYSICS2D_LINEAR_VECTOR2
#include "include/math/math.h"
#include "include/common/common.h"
namespace Physics2D
{
	struct Vector2
	{
		Vector2(const real& _x = 0.0f, const real& _y = 0.0f)
		{
			x = _x;
			y = _y;
		}
		Vector2(const Vector2& copy)
		{
			x = copy.x;
			y = copy.y;
		}
		Vector2& operator=(const Vector2& copy)
		{
			x = copy.x;
			y = copy.y;
			return *this;
		}
		Vector2(Vector2&& other) = default;

		Vector2 operator+(const Vector2& rhs)const
		{
			return Vector2(x + rhs.x, y + rhs.y);
		}

		Vector2 operator-(const Vector2& rhs)const
		{
			return Vector2(x - rhs.x, y - rhs.y);
		}

		Vector2 operator*(const int& factor)const
		{
			return Vector2(x * factor, y * factor);
		}

		Vector2 operator*(const real& factor)const
		{
			return Vector2(x * factor, y * factor);
		}

		Vector2 operator/(const real& factor)const
		{
			assert(!realEqual(factor, 0));
			return Vector2(x / factor, y / factor);
		}

		Vector2 operator/(const int& factor)const
		{
			assert(!realEqual(factor, 0));
			return Vector2(x / factor, y / factor);
		}

		Vector2& operator+=(const Vector2& rhs)
		{
			x += rhs.x;
			y += rhs.y;
			return *this;
		}

		Vector2& operator-=(const Vector2& rhs)
		{
			x -= rhs.x;
			y -= rhs.y;
			return *this;
		}

		Vector2& operator*=(const real& factor)
		{
			x *= factor;
			y *= factor;
			return *this;
		}

		Vector2& operator*=(const int& factor)
		{
			x *= factor;
			y *= factor;
			return *this;
		}

		Vector2& operator/=(const real& factor)
		{
			assert(!realEqual(factor, 0));
			x /= factor;
			y /= factor;
			return *this;
		}

		Vector2& operator/=(const int& factor)
		{
			assert(!realEqual(factor, 0));
			x /= factor;
			y /= factor;
			return *this;
		}

		bool operator==(const Vector2& rhs)const
		{
			return x == rhs.x && y == rhs.y;
		}

		bool operator!=(const Vector2& rhs)const
		{
			return x != rhs.x || y != rhs.y;
		}

		real lengthSquare()const
		{
			return x * x + y * y;
		}

		real length()const
		{
			return sqrt(lengthSquare());
		}

		Vector2& set(const real& _x, const real& _y)
		{
			x = _x;
			y = _y;
			return *this;
		}

		Vector2& set(const Vector2& copy)
		{
			x = copy.x;
			y = copy.y;
			return *this;
		}

		Vector2& clear()
		{
			x = 0.0f;
			y = 0.0f;
			return *this;
		}

		Vector2& negate()
		{
			x *= -1;
			y *= -1;
			return *this;
		}

		Vector2& swap(Vector2& other) noexcept
		{
			realSwap(x, other.x);
			realSwap(y, other.y);
			return *this;
		}

		Vector2& normalize()
		{
			const real length_inv = Math::fastInverseSqrt<real>(lengthSquare());
			x *= length_inv;
			y *= length_inv;
			return *this;
		}

		Vector2 normal()const
		{
			return Vector2(*this).normalize();
		}

		bool equal(const Vector2& rhs)const
		{
			return realEqual(x, rhs.x) && realEqual(y, rhs.y);
		}

		real dot(const Vector2& rhs)const
		{
			return x * rhs.x + y * rhs.y;
		}

		real cross(const Vector2& rhs)const
		{
			return x * rhs.y - y * rhs.x;
		}

		Vector2 perpendicular()const
		{
			return Vector2(-y, x);
		}
		static real dotProduct(const Vector2& lhs, const Vector2& rhs)
		{
			return lhs.x * rhs.x + lhs.y * rhs.y;
		}
		static real crossProduct(const Vector2& lhs, const Vector2& rhs)
		{
			return lhs.x * rhs.y - lhs.y * rhs.x;
		}
		static real crossProduct(const real& x1, const real& y1, const real& x2, const real& y2)
		{
			return x1 * y2 - x2 * y1;
		}
		static Vector2 crossProduct(const real& lhs, const Vector2& rhs)
		{
			return Vector2(-lhs * rhs.y, lhs * rhs.x);
		}
		static Vector2 crossProduct(const Vector2& lhs, const real& rhs)
		{
			return Vector2(rhs * lhs.y, -rhs * lhs.x);
		}
		real x;
		real y;
	};
}
#endif