#ifndef PHYSICS2D_SHAPE_SECTOR_H
#define PHYSICS2D_SHAPE_SECTOR_H
#include "polygon.h"
namespace Physics2D
{
    class Sector : public Polygon
    {
    public:
        Sector();
        
        real startRadian()const;
        real spanRadian()const;
        real radius()const;
        real area()const;
        real samplePoints()const;

        void setSamplePoints(const real& points);
        void setStartRadian(const real& radian);
        void setSpanRadian(const real& radian);
        void setRadius(const real& radius);
        void set(const real& start, const real& end, const real& radius);
    private:
        void translateVertices();
        real m_startRadian;
        real m_spanRadian;
        real m_radius;
        real m_samplePoints;
    };
}
#endif