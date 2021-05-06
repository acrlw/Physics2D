#ifndef PHYSICS2D_SAT_H
#define PHYSICS2D_SAT_H

#include "include/collision/contact.h"
#include "include/geometry/shape.h"
namespace Physics2D
{
    /// <summary>
    /// Separating Axis Theorem
    /// </summary>
    
    class SAT
    {
    public:
        static std::optional<PenetrationInfo> circleVsCircle(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static std::optional<Vector2> circleVsPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static std::optional<Vector2> polygonVsPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
    private:
        struct ProjectedPoint
        {
            Vector2 vertex;
            real value = 0;
        };
    	struct ProjectedSegment
    	{
            ProjectedPoint min;
            ProjectedPoint max;
            static std::tuple<ProjectedSegment, real> intersect(const ProjectedSegment& s1, const ProjectedSegment& s2);
    	};
    };
}
#endif
