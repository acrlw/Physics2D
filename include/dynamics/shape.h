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
		inline Type type()
		{
			return m_type;
		}
		virtual ~Shape() {};
	protected:
		Type m_type;
	};
	/// <summary>
	/// Convex polygon, not concave!
	/// </summary>
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
		void append(const Vector2& vertex)
		{
			m_vertices.emplace_back(vertex);
		}
		static Vector2 triangleGravityPoint(const Vector2& a1, const Vector2& a2, const Vector2& a3)
		{
			return Vector2(a1 + a2 + a3) / 3;
		}

		static number triangleArea(const Vector2& a1, const Vector2& a2, const Vector2& a3)
		{
			return abs(Vector2::crossProduct(a1 - a2, a1 - a3)) / 2;
		}
		static Vector2 calculateCenter(const Polygon& polygon)
		{
			if (polygon.vertices().size() >= 4)
			{
				Vector2 pos;
				number area = 0;
				size_t p_a, p_b, p_c;
				p_a = 0, p_b = 0, p_c = 0;
				for (size_t i = 0; i < polygon.vertices().size() - 1; i++)
				{
					p_b = i + 1;
					p_c = i + 2;
					if (p_b == polygon.vertices().size() - 2)
						break;
					number a = triangleArea(polygon.vertices()[p_a], polygon.vertices()[p_b], polygon.vertices()[p_c]);
					Vector2 p = triangleGravityPoint(polygon.vertices()[p_a], polygon.vertices()[p_b], polygon.vertices()[p_c]);
					pos += p * a;
					area += a;
				}
				pos /= area;
				return pos;
			}
		}
		Vector2 center()const
		{
			return calculateCenter(*this);
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
		void set(const number& width, const number& height)
		{
			m_width = width;
			m_height = height;
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
