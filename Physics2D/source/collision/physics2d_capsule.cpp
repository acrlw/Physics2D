#include "physics2d_capsule.h"
namespace Physics2D
{

	Capsule::Capsule(real width, real height) : m_halfWidth(width / 2.0f), m_halfHeight(height / 2.0f)
	{
		m_type = Type::Capsule;
	}
	bool Capsule::contains(const Vector2& point, const real& epsilon)
	{
		real r = 0, h = 0;
		Vector2 anchorPoint1, anchorPoint2;
		if (m_halfWidth >= m_halfHeight)//Horizontal
		{
			r = m_halfHeight;
			h = m_halfWidth - m_halfHeight;
			
			anchorPoint1.set(h, 0);
			anchorPoint2.set(-h, 0);
			if (point.x - anchorPoint1.x <= epsilon && point.x - anchorPoint2.x >= epsilon
				&& point.y - r <= epsilon && point.y + r >= epsilon)
				return true;
		}
		else//Vertical
		{
			r = m_halfWidth / 2;
			h = m_halfHeight - m_halfWidth;
			anchorPoint1.set(0, h);
			anchorPoint2.set(0, -h);

			if (point.y - anchorPoint1.y <= epsilon && point.y - anchorPoint2.y >= epsilon
				&& point.x - r <= epsilon && point.x + r >= epsilon)
				return true;
		}
		if ((anchorPoint1 - point).lengthSquare() - r * r <= epsilon ||
			(anchorPoint2 - point).lengthSquare() - r * r <= epsilon)
			return true;

		return false;
	}

	void Capsule::scale(const real& factor)
	{
		m_halfWidth *= factor;
		m_halfHeight *= factor;
	}

	Vector2 Capsule::center() const
	{
		return Vector2();
	}

	void Capsule::set(real width, real height)
	{
		m_halfWidth = width / 2.0f;
		m_halfHeight = height / 2.0f;
	}

	void Capsule::setWidth(real width)
	{
		m_halfWidth = width * 2.0f;
	}

	void Capsule::setHeight(real height)
	{
		m_halfHeight = height * 2.0f;
	}

	real Capsule::width()const
	{
		return 2.0f * m_halfWidth;
	}

	real Capsule::height()const
	{
		return 2.0f * m_halfHeight;
	}

	real Capsule::halfWidth() const
	{
		return m_halfWidth;
	}

	real Capsule::halfHeight() const
	{
		return m_halfHeight;
	}

	Vector2 Capsule::topLeft() const
	{
		Vector2 result;
		real r;
		if (m_halfWidth >= m_halfHeight)//Horizontal
		{
			r = m_halfHeight;
			result.set(-m_halfWidth + r, r);
		}
		else//Vertical
		{
			r = m_halfWidth;
			result.set(-r, m_halfHeight - r);
		}
		return result;
	}
	Vector2 Capsule::bottomLeft() const
	{
		return -topRight();
	}

	Vector2 Capsule::topRight() const
	{
		Vector2 result;
		real r;
		if (m_halfWidth >= m_halfHeight)//Horizontal
		{
			r = m_halfHeight;
			result.set(m_halfWidth - r, r);
		}
		else//Vertical
		{
			r = m_halfWidth;
			result.set(r, m_halfHeight - r);
		}
		return result;
	}

	Vector2 Capsule::bottomRight() const
	{
		return -topLeft();
	}

	Container::Vector<Vector2> Capsule::boxVertices() const
	{
		Container::Vector<Vector2> vertices;
		vertices.reserve(4);
		vertices.emplace_back(this->topLeft());
		vertices.emplace_back(this->bottomLeft());
		vertices.emplace_back(this->bottomRight());
		vertices.emplace_back(this->topRight());
		return vertices;
	}
}