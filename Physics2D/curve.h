#ifndef PHYSICS2D_SHAPE_CURVE_H
#define PHYSICS2D_SHAPE_CURVE_H
#include "shape.h"
namespace Physics2D
{
    class Curve : public Shape
    {

    public:
        Curve();
        void set(const Vector2& start, const Vector2& control1, const Vector2& control2, const Vector2& end);

        Vector2 startPoint() const;
        void setStartPoint(const Vector2& startPoint);

        Vector2 control1() const;
        void setControl1(const Vector2& control1);

        Vector2 control2() const;
        void setControl2(const Vector2& control2);

        Vector2 endPoint() const;
        void setEndPoint(const Vector2& endPoint);

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