#ifndef PHYSICS2D_LINEAR_VECTOR3
#define PHYSICS2D_LINEAR_VECTOR3
#include "include/math/math.h"
#include "include/common/common.h"
namespace Physics2D
{

	struct Vector3
	{
		Vector3(const number& _x = 0.0f, const number& _y = 0.0f, const number& _z = 0.0f)
		{
			x = _x;
			y = _y;
			z = _z;
		}
		Vector3(const Vector3& copy)
		{
			x = copy.x;
			y = copy.y;
			z = copy.z;
		}
		Vector3& operator=(const Vector3& copy)
		{
			x = copy.x;
			y = copy.y;
			z = copy.z;
			return *this;
		}
		Vector3(Vector3&& other) = default;

		Vector3 operator+(const Vector3& rhs)const
		{
			return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
		}

		Vector3 operator-(const Vector3& other)const
		{
			return Vector3(x - other.x, y - other.y, z - other.z);
		}

		Vector3 operator*(const number& factor)const
		{
			return Vector3(x * factor, y * factor, z * factor);
		}

		Vector3 operator*(const int& factor)const
		{
			return Vector3(x * factor, y * factor, z * factor);
		}

		Vector3 operator/(const number& factor)const
		{
			assert(numberEqual(factor, 0));
			return Vector3(x / factor, y / factor, z / factor);
		}

		Vector3 operator/(const int& factor)const
		{
			assert(numberEqual(factor, 0));
			return Vector3(x / factor, y / factor, z / factor);
		}

		Vector3& operator+=(const Vector3& rhs)
		{
			x += rhs.x;
			y += rhs.y;
			z += rhs.z;
			return *this;
		}

		Vector3& operator-=(const Vector3& rhs)
		{
			x -= rhs.x;
			y -= rhs.y;
			z -= rhs.z;
			return *this;
		}

		Vector3& operator*=(const number& factor)
		{
			x *= factor;
			y *= factor;
			z *= factor;
			return *this;
		}

		Vector3& operator*=(const int& factor)
		{
			x *= factor;
			y *= factor;
			z *= factor;
			return *this;
		}

		Vector3& operator/=(const number& factor)
		{
			assert(numberEqual(factor, 0));
			x /= factor;
			y /= factor;
			z /= factor;
			return *this;
		}

		Vector3& operator/=(const int& factor)
		{
			assert(numberEqual(factor, 0));
			x /= factor;
			y /= factor;
			z /= factor;
			return *this;
		}

		Vector3& set(const number& _x, const number& _y, const number& _z)
		{
			x = _x;
			y = _y;
			z = _z;
			return *this;
		}

		Vector3& set(const Vector3& other)
		{
			x = other.x;
			y = other.y;
			z = other.z;
			return *this;
		}

		Vector3& clear()
		{
			x = 0.0f;
			y = 0.0f;
			z = 0.0f;
			return *this;
		}

		Vector3& negate()
		{
			x *= -1;
			y *= -1;
			z *= -1;
			return *this;
		}

		number lengthSquare()const
		{
			return x * x + y * y + z * z;
		}

		number length()const
		{
			return sqrt(lengthSquare());
		}

		Vector3& normalize()
		{
			const number length_inv = fastInverseSqrt<number>(lengthSquare());
			x *= length_inv;
			y *= length_inv;
			z *= length_inv;
			return *this;
		}

		Vector3 normal()const
		{
			return Vector3(*this).normalize();
		}

		bool equal(const Vector3& rhs)const
		{
			return numberEqual(x, rhs.x) && numberEqual(y, rhs.y) && numberEqual(z, rhs.z);
		}

		Vector3& swap(Vector3& other)
		{
			numberSwap(x, other.x);
			numberSwap(y, other.y);
			numberSwap(z, other.z);
			return *this;
		}

		number dot(const Vector3& rhs)const
		{
			return x * rhs.x + y * rhs.y + z * rhs.z;
		}

		Vector3& cross(const Vector3& rhs)
		{
			x = y * rhs.z - rhs.y * z;
			y = rhs.x * z - x * rhs.z;
			z = x * rhs.y - y * rhs.x;
			return *this;
		}
		static number dotProduct(const Vector3& lhs, const Vector3& rhs)
		{
			return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
		}
		static Vector3 crossProduct(const Vector3& lhs, const Vector3& rhs)
		{
			return Vector3(lhs.y * rhs.z - rhs.y * lhs.z, rhs.x * lhs.z - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x);
		}
		number x;
		number y;
		number z;
	};
}
#endif