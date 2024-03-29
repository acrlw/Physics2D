#ifndef PHYSICS2D_SHAPE_POLYGON_H
#define PHYSICS2D_SHAPE_POLYGON_H
#include "physics2d_shape.h"
namespace Physics2D
{
    class PHYSICS2D_API Polygon : public Shape
    {

    public:
        Polygon();

        const Container::Vector<Vector2>& vertices() const;
        void append(const std::initializer_list<Vector2>& vertices);
        void append(const Vector2& vertex);
        Vector2 center()const override;
        void scale(const real& factor) override;
        bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) override;
    protected:
        Container::Vector<Vector2> m_vertices;
        void updateVertices();
    };
}
#endif