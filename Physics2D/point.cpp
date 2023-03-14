#include "point.h"
namespace Physics2D
{
	Point::Point()
	{
		m_type = Type::Point;
	}

	Vector2 Point::position() const
	{
		return m_position;
	}

	void Point::scale(const real& factor)
	{
		m_position *= factor;
	}
	bool Point::contains(const Vector2& point, const real& epsilon)
	{
		return (m_position - point).lengthSquare() < epsilon;
	}
	void Point::setPosition(const Vector2& pos)
	{
		m_position = pos;
	}

	Vector2 Point::center()const
	{
		return m_position;
	}
}