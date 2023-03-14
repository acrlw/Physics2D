#include "vector4.h"
#include "math.h"

namespace Physics2D
{

	Vector4::Vector4(const real& _x, const real& _y, const real& _z, const real& _w)
		: x(_x), y(_y), z(_z), w(_w)
	{
	}

	Vector4& Vector4::operator=(const Vector4& copy)
	{
		if (&copy == this)
			return *this;
		x = copy.x;
		y = copy.y;
		z = copy.z;
		w = copy.w;
		return *this;
	}

	Vector4& Vector4::operator=(const Vector3& copy)
	{
		x = copy.x;
		y = copy.y;
		z = copy.z;
		w = 0.0;
		return *this;
	}

	Vector4::Vector4(const Vector3& copy)
		:x(copy.x), y(copy.y), z(copy.z), w(0.0)
	{

	}

	Vector4 Vector4::operator+(const Vector4& rhs) const
	{
		return Vector4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
	}

	Vector4 Vector4::operator-(const Vector4& other) const
	{
		return Vector4(x - other.x, y - other.y, z - other.z, w - other.w);
	}
	Vector4 Vector4::operator-() const
	{
		return Vector4(-x, -y, -z, -w);
	}
	Vector4 Vector4::operator*(const real& factor) const
	{
		return Vector4(x * factor, y * factor, z * factor, w * factor);
	}

	Vector4 Vector4::operator/(const real& factor) const
	{

		assert(!realEqual(factor, 0));
		return Vector3(x / factor, y / factor, z / factor);
	}

	Vector4& Vector4::operator+=(const Vector4& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		w += rhs.w;
		return *this;
	}

	Vector4& Vector4::operator-=(const Vector4& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		w -= rhs.w;
		return *this;
	}

	Vector4& Vector4::operator*=(const real& factor)
	{
		x *= factor;
		y *= factor;
		z *= factor;
		w *= factor;
		return *this;
	}

	Vector4& Vector4::operator/=(const real& factor)
	{
		assert(!realEqual(factor, 0));
		x /= factor;
		y /= factor;
		z /= factor;
		w /= factor;
		return *this;
	}

	Vector4& Vector4::set(const real& _x, const real& _y, const real& _z, const real& _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
		return *this;
	}
	

	Vector4& Vector4::set(const Vector4& other)
	{
		x = other.x;
		y = other.y;
		z = other.z;
		w = other.w;
		return *this;
	}

	Vector4& Vector4::set(const Vector3& other)
	{
		x = other.x;
		y = other.y;
		z = other.z;
		w = 0.0;
		return *this;
	}

	Vector4& Vector4::clear()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
		w = 0.0;
		return *this;
	}

	Vector4& Vector4::negate()
	{
		x = -x;
		y = -y;
		z = -z;
		w = -w;
		return *this;
	}

	Vector4& Vector4::normalize()
	{
		const real length_inv = Math::fastInverseSqrt<real>(lengthSquare());
		x *= length_inv;
		y *= length_inv;
		z *= length_inv;
		w *= length_inv;
		return *this;
	}

	real Vector4::lengthSquare() const
	{
		return x * x + y * y + z * z + w * w;
	}

	real Vector4::length() const
	{
		return sqrt(lengthSquare());
	}

	Vector4 Vector4::normal() const
	{
		return Vector4(*this).normalize();
	}

	Vector4 Vector4::negative() const
	{
		return Vector4(-x, -y, -z, -w);
	}

	bool Vector4::equal(const Vector4& rhs) const
	{
		return realEqual(x, rhs.x) && realEqual(y, rhs.y)
		&& realEqual(z, rhs.z) && realEqual(w, rhs.w);
	}

	bool Vector4::fuzzyEqual(const Vector4& rhs, const real& epsilon) const
	{
		return (*this - rhs).lengthSquare() < epsilon;
	}

	bool Vector4::isOrigin(const real& epsilon) const
	{
		return fuzzyEqual({ 0, 0, 0 , 0}, epsilon);
	}

	Vector4& Vector4::swap(Vector4& other)
	{
		realSwap(x, other.x);
		realSwap(y, other.y);
		realSwap(z, other.z);
		realSwap(w, other.w);
		return *this;
	}

	real Vector4::dot(const Vector4& rhs) const
	{
		return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
	}

	real Vector4::dotProduct(const Vector4& lhs, const Vector4& rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
	}
	
}
