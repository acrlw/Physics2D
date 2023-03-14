#ifndef PHYSICS2D_SHAPE_H
#define PHYSICS2D_SHAPE_H
#include "linear.h"
#include "common.h"

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
            	Capsule,
                Edge,
                Curve,
                Sector
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
		Shape* shape;
        Vector2 transform;
        real rotation = 0;
        Vector2 translate(const Vector2& source)const
        {
            return Matrix2x2(rotation).multiply(source) + transform;
        }
    };
}
#endif
