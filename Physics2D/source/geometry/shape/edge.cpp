#include "../../../include/geometry/shape/edge.h"
#include "../../../include/geometry/algorithm/2d.h"
namespace Physics2D
{
	Edge::Edge()
	{
		m_type = Type::Edge;
	}

	void Edge::set(const Vector2& start, const Vector2& end)
	{
		m_startPoint = start;
		m_endPoint = end;
		m_normal = (m_endPoint - m_startPoint).perpendicular().normal().negate();
	}

	void Edge::setStartPoint(const Vector2& start)
	{
		m_startPoint = start;
	}

	void Edge::setEndPoint(const Vector2& end)
	{
		m_endPoint = end;
	}

	Vector2 Edge::startPoint() const
	{
		return m_startPoint;
	}

	Vector2 Edge::endPoint() const
	{
		return m_endPoint;
	}

	void Edge::scale(const real& factor)
	{
		m_startPoint *= factor;
		m_endPoint *= factor;
	}

	bool Edge::contains(const Vector2& point, const real& epsilon)
	{
		return GeometryAlgorithm2D::isPointOnSegment(m_startPoint, m_endPoint, point);
	}

	Vector2 Edge::center()const
	{
		return (m_startPoint + m_endPoint) / 2.0f;
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