
#include "sector.h"
namespace Physics2D
{
	Sector::Sector()
	{
		m_startRadian = 0;
		m_spanRadian = 0;
		m_radius = 0;
		m_samplePoints = 8;
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

	real Sector::samplePoints() const
	{
		return m_samplePoints;
	}

	void Sector::setSamplePoints(const real& points)
	{
		m_samplePoints = points;
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
		translateVertices();
	}

	void Sector::translateVertices()
	{
		if (m_spanRadian <= 0 && m_samplePoints < 3)
			return;
		m_vertices.emplace_back(Vector2{ 0, 0 });
		real step = m_spanRadian / m_samplePoints;
		for(real i = m_startRadian; i <= m_startRadian + m_spanRadian - step; )
		{
			m_vertices.emplace_back(Matrix2x2(i).multiply(Vector2(m_radius, 0)));
			i += step;
		}
		m_vertices.emplace_back(Matrix2x2(m_startRadian + m_spanRadian).multiply(Vector2(m_radius, 0)));
		m_vertices.emplace_back(Vector2{ 0, 0 });
		updateVertices();
	}
}
