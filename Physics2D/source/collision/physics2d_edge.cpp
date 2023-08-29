#include "physics2d_edge.h"
#include "physics2d_algorithm_2d.h"
namespace Physics2D
{
	Edge::Edge()
	{
		m_type = Type::Edge;
	}

	void Edge::set(const Vector2& start, const Vector2& end)
	{
		m_point[0] = start;
		m_point[1] = end;
		m_normal = (m_point[1] - m_point[0]).perpendicular().normal().negate();
	}

	void Edge::setStartPoint(const Vector2& start)
	{
		m_point[0] = start;
	}

	void Edge::setEndPoint(const Vector2& end)
	{
		m_point[1] = end;
	}

	Vector2 Edge::startPoint() const
	{
		return m_point[0];
	}

	Vector2 Edge::endPoint() const
	{
		return m_point[1];
	}

	void Edge::scale(const real& factor)
	{
		m_point[0] *= factor;
		m_point[1] *= factor;
	}

	bool Edge::contains(const Vector2& point, const real& epsilon)
	{
		return GeometryAlgorithm2D::isPointOnSegment(m_point[0], m_point[1], point);
	}

	Vector2 Edge::center()const
	{
		return (m_point[0] + m_point[1]) / 2.0f;
	}

	Vector2 Edge::normal() const
	{
		return m_normal;
	}

	void Edge::setNormal(const Vector2& normal)
	{
		m_normal = normal;
	}
}