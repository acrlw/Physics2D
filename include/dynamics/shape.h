#ifndef PHYSICS2D_SHAPE_H
#define PHYSICS2D_SHAPE_H
#include "include/math/math.h"
#include "include/common/common.h"
#include "include/math/algorithm/graphics/2d.h"
namespace Physics2D
{
    using Point2 = Vector2;
    class Shape
    {
        public:
            enum class Type
                {
                Point,
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
    class Point: public Shape
    {
        public:

            Point()
            {
                m_type = Type::Point;
            }

            Vector2 position() const
            {
                return m_position;
            }

            void setPosition(const Vector2& pos)
            {
                m_position = pos;
            }
        private:
            Vector2 m_position;
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
            Vector2 center()const
            {
                return GeometryAlgorithm2D::calculateCenter(this->vertices());
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
            void setWidth(const number& width)
            {
                m_width = width;
            }
            void setHeight(const number& height)
            {
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
            void set(const Vector2& _start, const Vector2& _end)
            {
                m_startPoint = _start;
                m_endPoint = _end;
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
#endif
