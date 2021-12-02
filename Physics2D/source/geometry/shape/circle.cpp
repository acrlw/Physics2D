#include "../../../include/geometry/shape/circle.h"
namespace Physics2D
{
	Circle::Circle(real radius) : m_radius(radius)
	{
		m_type = Type::Circle;
	}

	real Circle::radius() const
	{
		return m_radius;
	}

	void Circle::setRadius(const real& radius)
	{
		m_radius = radius;
	}

	void Circle::scale(const real& factor)
	{
		m_radius *= factor;
	}

	bool Circle::contains(const Vector2& point, const real& epsilon)
	{
		return (m_radius * m_radius - point.lengthSquare()) > epsilon;
	}

	Vector2 Circle::center()const
	{
		return Vector2();
	}
}