#ifndef PHYSICS2D_SHAPE_CAPSULE_H
#define PHYSICS2D_SHAPE_CAPSULE_H
#include "shape.h"
namespace Physics2D
{
    class Capsule : public Shape
    {
    public:
        Capsule(real width = 0.0f, real height = 0.0f);
        bool contains(const Vector2& point, const real& epsilon) override;
        void scale(const real& factor) override;
        Vector2 center() const override;
        void set(real width, real height);
        void setWidth(real width);
        void setHeight(real height);
        real width()const;
        real height()const;
        Vector2 topLeft()const;
        Vector2 bottomLeft()const;
        Vector2 topRight()const;
        Vector2 bottomRight()const;
        Container::Vector<Vector2> boxVertices()const;
    private:
        real m_width;
        real m_height;
    };
}
#endif