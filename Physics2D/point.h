#ifndef PHYSICS2D_SHAPE_POINT_H
#define PHYSICS2D_SHAPE_POINT_H
#include "shape.h"
namespace Physics2D
{
    class Point : public Shape
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
}
#endif