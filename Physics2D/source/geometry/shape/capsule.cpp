#include "../../../include/geometry/shape/capsule.h"
namespace Physics2D
{

	Capsule::Capsule(real width, real height) : m_width(width), m_height(height)
	{
		m_type = Type::Capsule;
	}
	bool Capsule::contains(const Vector2& point, const real& epsilon)
	{
		real r = 0, h = 0;
		Vector2 anchorPoint1, anchorPoint2;
		if (m_width >= m_height)//Horizontal
		{
			r = m_height / 2;
			h = m_width - m_height;
			anchorPoint1.set(h / 2, 0);
			anchorPoint2.set(-h / 2, 0);
			if (point.x - anchorPoint1.x <= epsilon && point.x - anchorPoint2.x >= epsilon
				&& point.y - r <= epsilon && point.y + r >= epsilon)
				return true;
		}
		else//Vertical
		{
			r = m_width / 2;
			h = m_height - m_width;
			anchorPoint1.set(0, h / 2);
			anchorPoint2.set(0, -h / 2);

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
		m_width *= factor;
		m_height *= factor;
	}

	Vector2 Capsule::center() const
	{
		return Vector2();
	}

	void Capsule::set(real width, real height)
	{
		m_width = width;
		m_height = height;
	}

	void Capsule::setWidth(real width)
	{
		m_width = width;
	}

	void Capsule::setHeight(real height)
	{
		m_height = height;
	}

	real Capsule::width()const
	{
		return m_width;
	}

	real Capsule::height()const
	{
		return m_height;
	}

	Vector2 Capsule::topLeft() const
	{
		Vector2 result;
		real r;
		if (m_width >= m_height)//Horizontal
		{
			r = m_height / 2.0f;
			result.set(-m_width / 2.0f + r, r);
		}
		else//Vertical
		{
			r = m_width / 2.0f;
			result.set(-r, m_height / 2.0f - r);
		}
		return result;
	}
	Vector2 Capsule::bottomLeft() const
	{
		Vector2 result;
		real r;
		if (m_width >= m_height)//Horizontal
		{
			r = m_height / 2.0f;
			result.set(-m_width / 2.0f + r, -r);
		}
		else//Vertical
		{
			r = m_width / 2.0f;
			result.set(-r, -m_height / 2.0f + r);
		}
		return result;
	}

	Vector2 Capsule::topRight() const
	{
		return -bottomLeft();
	}

	Vector2 Capsule::bottomRight() const
	{
		return -topLeft();
	}

	std::vector<Vector2> Capsule::boxVertices() const
	{
		std::vector<Vector2> vertices;
		vertices.reserve(5);
		vertices.emplace_back(this->topLeft());
		vertices.emplace_back(this->bottomLeft());
		vertices.emplace_back(this->bottomRight());
		vertices.emplace_back(this->topRight());
		vertices.emplace_back(this->topLeft());
		return vertices;
	}
}