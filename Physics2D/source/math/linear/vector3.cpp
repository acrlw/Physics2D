#include "../../../include/math/linear/vector3.h"

namespace Physics2D
{
	Vector3::Vector3(const real& _x, const real& _y, const real& _z) : x(_x), y(_y), z(_z)
	{
	}

	Vector3::Vector3(const Vector3& copy) : x(copy.x), y(copy.y), z(copy.z)
	{
	}

	Vector3 Vector3::operator+(const Vector3& rhs) const
	{
		return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	Vector3 Vector3::operator-(const Vector3& other) const
	{
		return Vector3(x - other.x, y - other.y, z - other.z);
	}

	Vector3 Vector3::operator*(const real& factor) const
	{
		return Vector3(x * factor, y * factor, z * factor);
	}

	Vector3 Vector3::operator/(const real& factor) const
	{
		assert(!realEqual(factor, 0));
		return Vector3(x / factor, y / factor, z / factor);
	}

	Vector3& Vector3::operator+=(const Vector3& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}

	Vector3& Vector3::operator-=(const Vector3& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}

	Vector3& Vector3::operator*=(const real& factor)
	{
		x *= factor;
		y *= factor;
		z *= factor;
		return *this;
	}

	Vector3& Vector3::operator/=(const real& factor)
	{
		assert(!realEqual(factor, 0));
		x /= factor;
		y /= factor;
		z /= factor;
		return *this;
	}

	Vector3& Vector3::set(const real& _x, const real& _y, const real& _z)
	{
		x = _x;
		y = _y;
		z = _z;
		return *this;
	}

	Vector3& Vector3::set(const Vector3& other)
	{
		x = other.x;
		y = other.y;
		z = other.z;
		return *this;
	}

	Vector3& Vector3::clear()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
		return *this;
	}

	Vector3 Vector3::negative() const
	{
		return Vector3(-x, -y, -z);
	}
	Vector3& Vector3::negate()
	{
		x *= -1;
		y *= -1;
		z *= -1;
		return *this;
	}

	real Vector3::lengthSquare() const
	{
		return x * x + y * y + z * z;
	}

	real Vector3::length() const
	{
		return sqrt(lengthSquare());
	}

	Vector3& Vector3::normalize()
	{
		const real length_inv = Math::fastInverseSqrt<real>(lengthSquare());
		x *= length_inv;
		y *= length_inv;
		z *= length_inv;
		return *this;
	}

	Vector3 Vector3::normal() const
	{
		return Vector3(*this).normalize();
	}

	bool Vector3::equal(const Vector3& rhs) const
	{
		return realEqual(x, rhs.x) && realEqual(y, rhs.y) && realEqual(z, rhs.z);
	}

	bool Vector3::fuzzyEqual(const Vector3& rhs, const real& epsilon) const
	{
		return (*this - rhs).lengthSquare() < epsilon;
	}
	bool Vector3::isOrigin(const real& epsilon) const
	{
		return fuzzyEqual({0, 0, 0}, epsilon);
	}
	Vector3& Vector3::swap(Vector3& other)
	{
		realSwap(x, other.x);
		realSwap(y, other.y);
		realSwap(z, other.z);
		return *this;
	}

	real Vector3::dot(const Vector3& rhs) const
	{
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}

	Vector3& Vector3::cross(const Vector3& rhs)
	{
		x = y * rhs.z - rhs.y * z;
		y = rhs.x * z - x * rhs.z;
		z = x * rhs.y - y * rhs.x;
		return *this;
	}

	real Vector3::dotProduct(const Vector3& lhs, const Vector3& rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
	}

	Vector3 Vector3::crossProduct(const Vector3& lhs, const Vector3& rhs)
	{
		return Vector3(lhs.y * rhs.z - rhs.y * lhs.z, rhs.x * lhs.z - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x);
	}

	Vector3& Vector3::operator/=(const int& factor)
	{
		assert(!realEqual(factor, 0));
		x /= factor;
		y /= factor;
		z /= factor;
		return *this;
	}

	Vector3& Vector3::operator*=(const int& factor)
	{
		x *= factor;
		y *= factor;
		z *= factor;
		return *this;
	}

	Vector3 Vector3::operator/(const int& factor) const
	{
		assert(!realEqual(factor, 0));
		return Vector3(x / factor, y / factor, z / factor);
	}

	Vector3 Vector3::operator*(const int& factor) const
	{
		return Vector3(x * factor, y * factor, z * factor);
	}

	Vector3& Vector3::operator=(const Vector3& copy)
	{
		if (&copy == this)
			return *this;
		x = copy.x;
		y = copy.y;
		z = copy.z;
		return *this;
	}
}
