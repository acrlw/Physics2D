#ifndef PHYSICS2D_SHAPE_H
#define PHYSICS2D_SHAPE_H
#include "include/math/math.h"
#include "include/common/common.h"
#include "include/math/algorithm/graphics/2d.h"
namespace Physics2D
{
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
            virtual void scale(const number& factor) = 0;
            virtual ~Shape() {};
        protected:
            Type m_type;
    };

    /// <summary>
    /// Basic Shape Description Primitive.
    /// Including vertices/position/angle of shape
    /// </summary>
    struct ShapePrimitive
    {
        Shape* shape = nullptr;
        Vector2 transform;
        number rotation = 0;
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
            void scale(const number& factor) override
            {
                m_position *= factor;
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
    		void append(const std::initializer_list<Vector2>& vertices)
            {
	            for(const Vector2& vertex: vertices)
                    m_vertices.emplace_back(vertex);
            }
            void append(const Vector2& vertex)
            {
                m_vertices.emplace_back(vertex);
            }
            Vector2 center()const
            {
                return GeometryAlgorithm2D::calculateCenter(this->vertices());
            }
            void scale(const number& factor) override
            {
                assert(!m_vertices.empty());
                for (Vector2& vertex : m_vertices)
                    vertex *= factor;
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
            void set(const number& width, const number& height)
            {
                m_width = width;
                m_height = height;
                calcVertices();
            }
			number width()const
            {
                return m_width;
            }
            number height()const
            {
                return m_height;
            }
			void setWidth(const number& width)
            {
                m_width = width;
                calcVertices();
            }
            void setHeight(const number& height)
            {
                m_height = height;
                calcVertices();
            }
        private:
            void calcVertices()
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
			void setRadius(const number& radius)
            {
                m_radius = radius;
            }
            void scale(const number& factor) override
            {
                m_radius *= factor;
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
    		void set(const Vector2& leftTop, const Vector2& rightBottom)
            {
                m_width = rightBottom.x - leftTop.x;
                m_height = rightBottom.y - leftTop.y;
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

            void scale(const number& factor) override
            {
                m_width *= factor;
                m_height *= factor;
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
    	
            void set(const Vector2& start, const Vector2& end)
            {
                m_startPoint = start;
                m_endPoint = end;
            }
    		void setStartPoint(const Vector2& start)
            {
                m_startPoint = start;
            }
            void setEndPoint(const Vector2& end)
            {
                m_endPoint = end;
            }
            Vector2 startPoint()const
            {
                return m_startPoint;
            }
            Vector2 endPoint()const
            {
                return m_endPoint;
            }
            void scale(const number& factor) override
            {
                m_startPoint *= factor;
                m_endPoint *= factor;
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
    		void set(const Vector2& start, const Vector2& control1, const Vector2& control2, const Vector2& end)
            {
                m_startPoint = start;
                m_control1 = control1;
                m_control2 = control2;
                m_endPoint = end;
            }
            Vector2 startPoint() const
            {
                return m_startPoint;
            }

            void setStartPoint(const Vector2 &startPoint)
            {
                m_startPoint = startPoint;
            }

            Vector2 control1() const
            {
                return m_control1;
            }

            void setControl1(const Vector2 &control1)
            {
                m_control1 = control1;
            }

            Vector2 control2() const
            {
                return m_control2;
            }

            void setControl2(const Vector2 &control2)
            {
                m_control2 = control2;
            }

            Vector2 endPoint() const
            {
                return m_endPoint;
            }

            void setEndPoint(const Vector2 &endPoint)
            {
                m_endPoint = endPoint;
            }

            void scale(const number& factor) override
            {
                m_startPoint *= factor;
                m_control1 *= factor;
                m_control2 *= factor;
                m_endPoint *= factor;
            }
        private:
            Vector2 m_startPoint;
            Vector2 m_control1;
            Vector2 m_control2;
            Vector2 m_endPoint;
    };
}
#endif
