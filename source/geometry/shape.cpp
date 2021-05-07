#include "include/geometry/shape.h"
#include "include/geometry/algorithm/2d.h"

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

	Polygon::Polygon()
	{
		m_type = Type::Polygon;
	}

	const std::vector<Vector2>& Polygon::vertices() const
	{
		return m_vertices;
	}

	void Polygon::append(const std::initializer_list<Vector2>& vertices)
	{
		for (const Vector2& vertex : vertices)
			m_vertices.emplace_back(vertex);
	}

	void Polygon::append(const Vector2& vertex)
	{
		m_vertices.emplace_back(vertex);
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
		for(int i = 0;i < m_vertices.size() - 1;i++)
		{
			Vector2 ab = m_vertices[i + 1] - m_vertices[i];
			Vector2 ac = point - m_vertices[i];
			if (ab.cross(ac) < epsilon)
				return false;
		}
		return true;
	}

	Rectangle::Rectangle(const real& width, const real& height)
	{
		m_type = Type::Polygon;
		this->set(width, height);
	}

	void Rectangle::set(const real& width, const real& height)
	{
		m_width = width;
		m_height = height;
		calcVertices();
	}

	real Rectangle::width() const
	{
		return m_width;
	}

	real Rectangle::height() const
	{
		return m_height;
	}

	void Rectangle::setWidth(const real& width)
	{
		m_width = width;
		calcVertices();
	}

	void Rectangle::setHeight(const real& height)
	{
		m_height = height;
		calcVertices();
	}
	void Rectangle::scale(const real& factor)
	{
		m_width *= factor;
		m_height *= factor;
		calcVertices();
	}
	bool Rectangle::contains(const Vector2& point, const real& epsilon)
	{
		return (point.x < m_width / 2.0 && point.x > -m_width / 2.0) && 
			point.y < m_height / 2.0 && point.y > -m_height / 2.0;
	}
	void Rectangle::calcVertices()
	{
		m_vertices.clear();
		m_vertices.emplace_back(Vector2(-m_width * (0.5f), m_height * (0.5f)));
		m_vertices.emplace_back(Vector2(-m_width * (0.5f), -m_height * (0.5f)));
		m_vertices.emplace_back(Vector2(m_width * (0.5f), -m_height * (0.5f)));
		m_vertices.emplace_back(Vector2(m_width * (0.5f), m_height * (0.5f)));
		m_vertices.emplace_back(Vector2(-m_width * (0.5f), m_height * (0.5f)));
	}

	Circle::Circle() : m_radius(0)
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

	Ellipse::Ellipse(const real& width, const real& height)
	{
		m_type = Type::Ellipse;
		this->set(width, height);
	}

	void Ellipse::set(const Vector2& leftTop, const Vector2& rightBottom)
	{
		m_width = abs(rightBottom.x - leftTop.x);
		m_height = abs(rightBottom.y - leftTop.y);
	}

	void Ellipse::set(const real& width, const real& height)
	{
		m_width = width;
		m_height = height;
	}

	void Ellipse::setWidth(const real& width)
	{
		m_width = width;
	}

	void Ellipse::setHeight(const real& height)
	{
		m_height = height;
	}

	void Ellipse::scale(const real& factor)
	{
		m_width *= factor;
		m_height *= factor;
	}

	bool Ellipse::contains(const Vector2& point, const real& epsilon)
	{
		return false;
	}

	Vector2 Ellipse::center()const
	{
		return Vector2();
	}

	real Ellipse::width() const
	{
		return m_width;
	}

	real Ellipse::height() const
	{
		return m_height;
	}

	real Ellipse::A() const
	{
		return m_width * (0.5f);
	}

	real Ellipse::B() const
	{
		return m_height * (0.5f);
	}

	real Ellipse::C() const
	{
		real a = A();
		real b = B();
		return sqrt(a * a - b * b);
	}

	Edge::Edge()
	{
		m_type = Type::Edge;
	}

	void Edge::set(const Vector2& start, const Vector2& end)
	{
		m_startPoint = start;
		m_endPoint = end;
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
		return (m_startPoint + m_endPoint) / 2.0;
	}

	Curve::Curve()
	{
		m_type = Type::Curve;
	}

	void Curve::set(const Vector2& start, const Vector2& control1, const Vector2& control2, const Vector2& end)
	{
		m_startPoint = start;
		m_control1 = control1;
		m_control2 = control2;
		m_endPoint = end;
	}

	Vector2 Curve::startPoint() const
	{
		return m_startPoint;
	}

	void Curve::setStartPoint(const Vector2& startPoint)
	{
		m_startPoint = startPoint;
	}

	Vector2 Curve::control1() const
	{
		return m_control1;
	}

	void Curve::setControl1(const Vector2& control1)
	{
		m_control1 = control1;
	}

	Vector2 Curve::control2() const
	{
		return m_control2;
	}

	void Curve::setControl2(const Vector2& control2)
	{
		m_control2 = control2;
	}

	Vector2 Curve::endPoint() const
	{
		return m_endPoint;
	}

	void Curve::setEndPoint(const Vector2& endPoint)
	{
		m_endPoint = endPoint;
	}

	void Curve::scale(const real& factor)
	{
		m_startPoint *= factor;
		m_control1 *= factor;
		m_control2 *= factor;
		m_endPoint *= factor;
	}
	bool Curve::contains(const Vector2& point, const real& epsilon)
	{
		return false;
	}
	Vector2 Curve::center() const
	{
		return Vector2();
	}
	Vector2 ShapePrimitive::translate(const Vector2& source)const
	{
		return Matrix2x2(rotation).multiply(source) + transform;
	}
}
