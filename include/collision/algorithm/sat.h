#ifndef PHYSICS2D_SAT_H
#define PHYSICS2D_SAT_H

#include "include/collision/contact.h"
#include "include/geometry/shape.h"
namespace Physics2D
{
    struct ProjectedPoint
    {
        Vector2 vertex;
        real value = 0;
    };
    struct ProjectedEdge
    {
        Vector2 vertex1;
        Vector2 vertex2;
    };
    struct ProjectedSegment
    {
        ProjectedPoint min;
        ProjectedPoint max;
        static std::tuple<ProjectedSegment, real> intersect(const ProjectedSegment& s1, const ProjectedSegment& s2);
    };
	
    struct SATResult
    {
        ProjectedSegment pointPair;
        ProjectedEdge targetEdge;
        Vector2 normal;
        real penetration = Constant::Max;
        bool isColliding = false;
    };

    /// <summary>
    /// Separating Axis Theorem
    /// </summary>
    class SAT
    {
    public:
        static std::optional<PenetrationInfo> circleVsCircle(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static SATResult circleVsPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static SATResult polygonVsPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
    private:
        static ProjectedSegment axisProjection(const ShapePrimitive& shape, Polygon* polygon, const Vector2& normal);
    };
}
#endif
