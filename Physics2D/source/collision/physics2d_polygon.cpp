#include "physics2d_polygon.h"
#include "physics2d_algorithm_2d.h"
namespace Physics2D
{
	Polygon::Polygon()
	{
		m_type = Type::Polygon;
		m_vertices.reserve(4);
	}

	const Container::Vector<Vector2>& Polygon::vertices() const
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
		for(auto iter = m_vertices.begin(); iter != m_vertices.end(); ++iter)
		{
			auto next = iter + 1;
			if(next == m_vertices.end())
				next = m_vertices.begin();
			auto ref = next + 1;
			if(ref == m_vertices.end())
				ref = m_vertices.begin();
			if (!GeometryAlgorithm2D::isPointOnSameSide(*iter, *next, *ref, point))
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