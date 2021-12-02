#ifndef PHYSICS2D_SHAPE_SECTOR_H
#define PHYSICS2D_SHAPE_SECTOR_H
#include "../../geometry/shape.h"
namespace Physics2D
{
    class Sector : public Shape
    {
    public:
        Sector();
        bool contains(const Vector2& point, const real& epsilon) override;
        void scale(const real& factor) override;
        Vector2 center() const override;

        std::vector<Vector2> vertices()const;
        real startRadian()const;
        real spanRadian()const;
        real radius()const;
        real area()const;
        void setStartRadian(const real& radian);
        void setSpanRadian(const real& radian);
        void setRadius(const real& radius);
        void set(const real& start, const real& end, const real& radius);
    private:
        real m_startRadian;
        real m_spanRadian;
        real m_radius;
    };
}
#endif