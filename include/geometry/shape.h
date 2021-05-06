#ifndef PHYSICS2D_SHAPE_H
#define PHYSICS2D_SHAPE_H
#include "include/math/linear/linear.h"
#include "include/common/common.h"

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
            Type type()const
            {
                return m_type;
            }
            virtual void scale(const real& factor) = 0;
            virtual ~Shape() {};
            virtual bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) = 0;
            virtual Vector2 center()const = 0;
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
        real rotation = 0;
    };
    class Point: public Shape
    {
        public:

            Point();

            Vector2 position() const;
            void setPosition(const Vector2& pos);
            Vector2 center()const override;
            void scale(const real& factor) override;
            bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) override;
        private:
            Vector2 m_position;
    };
    /// <summary>
    /// Convex polygon, not concave!
    /// </summary>
    class Polygon: public Shape
    {

        public:
            Polygon();
    	
            const std::vector<Vector2>& vertices() const;
            void append(const std::initializer_list<Vector2>& vertices);
            void append(const Vector2& vertex);
            Vector2 center()const override;
            void scale(const real& factor) override;
            bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) override;
        protected:
            std::vector<Vector2> m_vertices;
    };
    class Rectangle: public Polygon
    {

        public:
            Rectangle(const real& width = 0, const real& height = 0);
            void set(const real& width, const real& height);

            real width()const;
            void setWidth(const real& width);

            real height()const;
            void setHeight(const real& height);

            void scale(const real& factor) override;
            bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) override;
        private:
            void calcVertices();
            real m_width;
            real m_height;
    };
    class Circle : public Shape
    {

        public:
            Circle();

            real radius() const;
            void setRadius(const real& radius);
            void scale(const real& factor) override;
            bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) override;
            Vector2 center()const override;
        private:
            real m_radius;
    };
    class Ellipse : public Shape
    {

        public:
            Ellipse(const real& width = 0, const real& height = 0);
            void set(const Vector2& leftTop, const Vector2& rightBottom);
            void set(const real& width, const real& height);

            real width()const;
            void setWidth(const real& width);

            real height()const;
            void setHeight(const real& height);

            void scale(const real& factor) override;
            bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) override;
            Vector2 center()const override;
            real A()const;
            real B()const;
            real C()const;
        private:
            real m_width;
            real m_height;
    };
    class Edge : public Shape
    {

        public:
            Edge();
    	
            void set(const Vector2& start, const Vector2& end);

            Vector2 startPoint()const;
            void setStartPoint(const Vector2& start);

            Vector2 endPoint()const;
            void setEndPoint(const Vector2& end);
            void scale(const real& factor) override;
            bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) override;
            Vector2 center()const override;
        private:
            Vector2 m_startPoint;
            Vector2 m_endPoint;
    };
    class Curve : public Shape
    {

        public:
            Curve();
            void set(const Vector2& start, const Vector2& control1, const Vector2& control2, const Vector2& end);

            Vector2 startPoint() const;
            void setStartPoint(const Vector2 &startPoint);

            Vector2 control1() const;
            void setControl1(const Vector2 &control1);

            Vector2 control2() const;
            void setControl2(const Vector2 &control2);

            Vector2 endPoint() const;
            void setEndPoint(const Vector2 &endPoint);

            void scale(const real& factor) override;
            bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) override;
            Vector2 center()const override;
        private:
            Vector2 m_startPoint;
            Vector2 m_control1;
            Vector2 m_control2;
            Vector2 m_endPoint;
    };
}
#endif
