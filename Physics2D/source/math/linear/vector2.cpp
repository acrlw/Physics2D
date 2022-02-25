#include "../../../include/math/linear/vector2.h"
#include "../../../include/math/math.h"
namespace Physics2D
{
	Vector2::Vector2(const real& _x, const real& _y) : x(_x), y(_y)
	{
	}

	Vector2::Vector2(const Vector2& copy) : x(copy.x), y(copy.y)
	{
	}

	Vector2 Vector2::operator+(const Vector2& rhs) const
	{
		return Vector2(x + rhs.x, y + rhs.y);
	}

	Vector2 Vector2::operator-(const Vector2& rhs) const
	{
		return Vector2(x - rhs.x, y - rhs.y);
	}
	Vector2 Vector2::operator-()const
	{
		return Vector2(-x, -y);
	}
	Vector2 Vector2::operator*(const int& factor) const
	{
		return Vector2(x * factor, y * factor);
	}

	Vector2 Vector2::operator/(const real& factor) const
	{
		assert(!realEqual(factor, 0));
		return Vector2(x / factor, y / factor);
	}

	Vector2& Vector2::operator+=(const Vector2& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}

	Vector2& Vector2::operator-=(const Vector2& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}

	Vector2& Vector2::operator*=(const real& factor)
	{
		x *= factor;
		y *= factor;
		return *this;
	}

	Vector2& Vector2::operator/=(const real& factor)
	{
		assert(!realEqual(factor, 0));
		x /= factor;
		y /= factor;
		return *this;
	}

	bool Vector2::operator==(const Vector2& rhs) const
	{
		return realEqual(x, rhs.x) && realEqual(y, rhs.y);
	}

	bool Vector2::operator!=(const Vector2& rhs) const
	{
		return !realEqual(x, rhs.x) || !realEqual(y, rhs.y);
	}

	real Vector2::lengthSquare() const
	{
		return x * x + y * y;
	}

	real Vector2::length() const
	{
		return std::sqrt(lengthSquare());
	}

	real Vector2::theta() const
	{
		return Math::arctanx(y, x);
	}

	Vector2& Vector2::set(const real& _x, const real& _y)
	{
		x = _x;
		y = _y;
		return *this;
	}

	Vector2& Vector2::set(const Vector2& copy)
	{
		x = copy.x;
		y = copy.y;
		return *this;
	}

	Vector2& Vector2::clear()
	{
		x = 0.0f;
		y = 0.0f;
		return *this;
	}

	Vector2& Vector2::negate()
	{
		x *= -1.0f;
		y *= -1.0f;
		return *this;
	}

	Vector2& Vector2::swap(Vector2& other) noexcept
	{
		realSwap(x, other.x);
		realSwap(y, other.y);
		return *this;
	}

	Vector2& Vector2::normalize()
	{
		const real length_inv = Math::fastInverseSqrt<real>(lengthSquare());
		x *= length_inv;
		y *= length_inv;
		return *this;
	}

	Vector2 Vector2::normal() const
	{
		return Vector2(*this).normalize();
	}

	Vector2 Vector2::negative() const
	{
		return Vector2(-x, -y);
	}

	bool Vector2::equal(const Vector2& rhs) const
	{
		return realEqual(x, rhs.x) && realEqual(y, rhs.y);
	}

	bool Vector2::fuzzyEqual(const Vector2& rhs, const real& epsilon)const
	{
		return fuzzyRealEqual(x, rhs.x, epsilon) && fuzzyRealEqual(y, rhs.y, epsilon);
	}

	bool Vector2::isOrigin(const real& epsilon) const
	{
		return fuzzyEqual({ 0, 0 }, epsilon);
	}

	real Vector2::dot(const Vector2& rhs) const
	{
		return x * rhs.x + y * rhs.y;
	}

	real Vector2::cross(const Vector2& rhs) const
	{
		return x * rhs.y - y * rhs.x;
	}

	Vector2 Vector2::perpendicular() const
	{
		return Vector2(-y, x);
	}

	real Vector2::dotProduct(const Vector2& lhs, const Vector2& rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y;
	}

	real Vector2::crossProduct(const Vector2& lhs, const Vector2& rhs)
	{
		return lhs.x * rhs.y - lhs.y * rhs.x;
	}

	real Vector2::crossProduct(const real& x1, const real& y1, const real& x2, const real& y2)
	{
		return x1 * y2 - x2 * y1;
	}

	Vector2 Vector2::crossProduct(const real& lhs, const Vector2& rhs)
	{
		return Vector2(-rhs.y, rhs.x) * lhs;
	}

	Vector2 Vector2::crossProduct(const Vector2& lhs, const real& rhs)
	{
		return Vector2(lhs.y, -lhs.x) * rhs;
	}

	Vector2& Vector2::operator/=(const int& factor)
	{
		assert(!realEqual(factor, 0));
		x /= factor;
		y /= factor;
		return *this;
	}

	Vector2& Vector2::operator*=(const int& factor)
	{
		x *= factor;
		y *= factor;
		return *this;
	}

	Vector2 Vector2::operator/(const int& factor) const
	{
		assert(!realEqual(factor, 0));
		return Vector2(x / factor, y / factor);
	}

	Vector2 Vector2::operator*(const real& factor) const
	{
		return Vector2(x * factor, y * factor);
	}

	Vector2& Vector2::operator=(const Vector2& copy)
	{
		if (&copy == this)
			return *this;
		x = copy.x;
		y = copy.y;
		return *this;
	}
}
