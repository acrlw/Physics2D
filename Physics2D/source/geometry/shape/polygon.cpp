#include "../../../include/geometry/shape/polygon.h"
#include "../../../include/geometry/algorithm/2d.h"
namespace Physics2D
{
	Polygon::Polygon()
	{
		m_type = Type::Polygon;
		m_vertices.reserve(4);
	}

	const std::vector<Vector2>& Polygon::vertices() const
	{
		return m_vertices;
	}

	void Polygon::append(const std::initializer_list<Vector2>& vertices)
	{
		for (const Vector2& vertex : vertices)
			m_vertices.emplace_back(vertex);
		updateVertices();
	}

	void Polygon::append(const Vector2& vertex)
	{
		m_vertices.emplace_back(vertex);
		updateVertices();
	}

	Vector2 Polygon::center()const
	{
		return GeometryAlgorithm2D::calculateCenter(this->vertices());
	}

	void Polygon::scale(const real& factor)
	{
		assert(!m_vertices.empty());
		for (Vector2& vertex : m_vertices)
			vertex *= factor;
	}

	bool Polygon::contains(const Vector2& point, const real& epsilon)
	{
		for (size_t i = 0; i <= m_vertices.size() - 2; i++)
		{
			Vector2 p1 = m_vertices[i];
			Vector2 p2 = m_vertices[i + 1];
			Vector2 ref = i + 2 == m_vertices.size() ? m_vertices[1] : m_vertices[i + 2];
			if (!GeometryAlgorithm2D::isPointOnSameSide(p1, p2, ref, point))
				return false;
		}
		return true;
	}

	void Polygon::updateVertices()
	{
		Vector2 center = this->center();
		for (auto& elem : m_vertices)
			elem -= center;
	}
}