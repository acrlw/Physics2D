#include "physics2d_ellipse.h"

namespace Physics2D
{
	Ellipse::Ellipse(const real& width, const real& height) : m_width(width), m_height(height)
	{
		m_type = Type::Ellipse;
	}

	void Ellipse::set(const Vector2& leftTop, const Vector2& rightBottom)
	{
		m_width = std::fabs(rightBottom.x - leftTop.x);
		m_height = std::fabs(rightBottom.y - leftTop.y);
	}

	void Ellipse::set(const real& width, const real& height)
	{
		m_width = width;
		m_height = height;
	}

	void Ellipse::setWidth(const real& width)
	{
		m_width = width;
	}

	void Ellipse::setHeight(const real& height)
	{
		m_height = height;
	}

	void Ellipse::scale(const real& factor)
	{
		m_width *= factor;
		m_height *= factor;
	}

	bool Ellipse::contains(const Vector2& point, const real& epsilon)
	{
		const real a = A();
		const real b = B();
		assert(!realEqual(a, 0) && !realEqual(b, 0));
		const real x = m_width > m_height ? point.x : point.y;
		const real y = m_width > m_height ? point.y : point.x;
		return (x / a) * (x / a) + (y / b) * (y / b) <= 1.0f;
	}

	Vector2 Ellipse::center() const
	{
		return Vector2();
	}

	real Ellipse::width() const
	{
		return m_width;
	}

	real Ellipse::height() const
	{
		return m_height;
	}

	real Ellipse::A() const
	{
		return m_width > m_height ? m_width / 2.0f : m_height / 2.0f;
	}

	real Ellipse::B() const
	{
		return m_width > m_height ? m_height / 2.0f : m_width / 2.0f;
	}

	real Ellipse::C() const
	{
		const real a = A();
		const real b = B();
		return sqrt(a * a - b * b);
	}
}
