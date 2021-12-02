#ifndef PHYSICS2D_SHAPE_EDGE_H
#define PHYSICS2D_SHAPE_EDGE_H
#include "../../geometry/shape.h"
namespace Physics2D
{
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
        Vector2 normal()const;
        void setNormal(const Vector2& normal);
    private:
        Vector2 m_startPoint;
        Vector2 m_endPoint;
        Vector2 m_normal;
    };
}
#endif