#pragma once
#include "include/math/math.h"
#include "include/common/common.h"
namespace Physics2D
{
	using Point2 = Vector2;
	class Shape
	{
	public:
		enum class Type
		{
			Polygon,
			Circle,
			Ellipse,
			Edge,
			Curve
		};
		inline Type type()const
		{
			return m_type;
		}
		virtual ~Shape() {};
	protected:
		Type m_type;
	};
	class Polygon: public Shape
	{

	public:
		Polygon()
		{
			m_type = Type::Polygon;
		}
		const std::vector<Vector2>& vertices() const
		{
			return m_vertices;
		}
	protected:
		std::vector<Vector2> m_vertices;
	};
	class Rectangle: public Polygon
	{
		
	public:
		Rectangle(const number& width = 0.0f, const number& height = 0.0f)
		{
			m_type = Type::Polygon;
			this->set(width, height);
		}
		inline void set(const number& width, const number& height)
		{
			m_width = width;
			m_height = height;
			calcVertices();
		}
	private:
		inline void calcVertices()
		{
			m_vertices.clear();
			m_vertices.emplace_back(Vector2(-m_width / 2.0f, m_height / 2.0f));
			m_vertices.emplace_back(Vector2(-m_width / 2.0f, -m_height / 2.0f));
			m_vertices.emplace_back(Vector2(m_width / 2.0f, -m_height / 2.0f));
			m_vertices.emplace_back(Vector2(m_width / 2.0f, m_height / 2.0f));
			m_vertices.emplace_back(Vector2(-m_width / 2.0f, m_height / 2.0f));
		}
		number m_width;
		number m_height;
	};
	class Circle : public Shape
	{

	public:
		Circle()
		{
			m_type = Type::Circle;
		}

		number radius() const
		{
			return m_radius;
		}
	private:
		number m_radius;
	};
	class Ellipse : public Shape
	{

	public:
		Ellipse()
		{
			m_type = Type::Ellipse;
		}
		number width()const
		{
			return m_width;
		}
		number height()const
		{
			return m_height;
		}
		number A()const
		{
			return m_width / 2;
		}
		number B()const
		{
			return m_height / 2;
		}
		number C()const
		{
			number a = A();
			number b = B();
			return sqrt(a * a - b * b);
		}
	private:
		number m_width;
		number m_height;
	};
	class Edge : public Shape
	{

	public:
		Edge()
		{
			m_type = Type::Edge;
		}
		Vector2 startPoint()const
		{
			return m_startPoint;
		}
		Vector2 endPoint()const
		{
			return m_endPoint;
		}
	private:
		Vector2 m_startPoint;
		Vector2 m_endPoint;
	};
	class Curve : public Shape
	{

	public:
		Curve()
		{
			m_type = Type::Curve;
		}
	private:
		Vector2 m_startPoint;
		Vector2 m_control1;
		Vector2 m_control2;
		Vector2 m_endPoint;
	};
}
