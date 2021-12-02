#include "../../../include/geometry/shape/sector.h"
namespace Physics2D
{
	Sector::Sector()
	{
		m_type = Shape::Type::Sector;
		m_startRadian = 0;
		m_spanRadian = 0;
		m_radius = 0;
	}
	bool Sector::contains(const Vector2& point, const real& epsilon)
	{
		real theta = point.theta();
		return theta >= m_startRadian && theta <= m_spanRadian + m_startRadian && point.lengthSquare() <= m_radius * m_radius;
	}
	void Sector::scale(const real& factor)
	{
		m_radius *= factor;
	}
	Vector2 Sector::center() const
	{
		Vector2 st = Matrix2x2(m_startRadian).multiply(Vector2{ m_radius, 0 });
		Vector2 ed = Matrix2x2(m_startRadian + m_spanRadian).multiply(Vector2{ m_radius, 0 });
		Vector2 normal = (st + ed) / 2;
		real c = (st - ed).length();
		real l = m_spanRadian * m_radius;
		normal.normalize();
		Vector2 result = normal * (2.0f * m_radius * c / (3.0f * l));
		return result;
	}

	std::vector<Vector2> Sector::vertices() const
	{
		std::vector<Vector2> vertices;
		vertices.emplace_back(Vector2{ 0, 0 });
		vertices.emplace_back(Matrix2x2(m_startRadian).multiply(Vector2(m_radius, 0)));
		vertices.emplace_back(Matrix2x2(m_startRadian + m_spanRadian).multiply(Vector2(m_radius, 0)));
		vertices.emplace_back(Vector2{ 0, 0 });
		return vertices;
	}

	real Sector::startRadian() const
	{
		return m_startRadian;
	}

	real Sector::spanRadian() const
	{
		return m_spanRadian;
	}
	real Sector::radius() const
	{
		return m_radius;
	}

	real Sector::area() const
	{
		return m_spanRadian * m_radius * m_radius / 2.0f;
	}

	void Sector::setStartRadian(const real& radian)
	{
		m_startRadian = radian;
	}

	void Sector::setSpanRadian(const real& radian)
	{
		m_spanRadian = radian;
	}

	void Sector::setRadius(const real& radius)
	{
		m_radius = radius;
	}
	void Sector::set(const real& start, const real& end, const real& radius)
	{
		m_startRadian = start;
		m_spanRadian = end;
		m_radius = radius;
	}
}