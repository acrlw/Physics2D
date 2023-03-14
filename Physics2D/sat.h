#ifndef PHYSICS2D_SAT_H
#define PHYSICS2D_SAT_H

#include "shape.h"
#include "clip.h"

namespace Physics2D
{
    struct ProjectedPoint
    {
        Vector2 vertex;
        real value = 0;
        int index = -1;
        bool operator==(const ProjectedPoint& rhs);
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
        PointPair contactPair[2];
        uint32_t contactPairCount = 0;
        Vector2 normal;
        real penetration = 0;
        bool isColliding = false;
    };

    /// <summary>
    /// Separating Axis Theorem
    /// </summary>
    class SAT
    {
    public:
        static SATResult circleVsCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static SATResult circleVsSector(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static SATResult circleVsEdge(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static SATResult circleVsCircle(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static SATResult circleVsPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);

        static SATResult polygonVsPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
		static SATResult polygonVsEdge(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static SATResult polygonVsCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static SATResult polygonVsSector(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        
        static SATResult capsuleVsEdge(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
        static SATResult capsuleVsCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
		static SATResult capsuleVsSector(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);

        static SATResult sectorVsSector(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);

        static ProjectedSegment axisProjection(const ShapePrimitive& shape, Polygon* polygon, const Vector2& normal);
        static ProjectedSegment axisProjection(const ShapePrimitive& shape, Circle* circle, const Vector2& normal);
        static ProjectedSegment axisProjection(const ShapePrimitive& shape, Ellipse* ellipse, const Vector2& normal);
        static ProjectedSegment axisProjection(const ShapePrimitive& shape, Capsule* capsule, const Vector2& normal);
        static ProjectedSegment axisProjection(const ShapePrimitive& shape, Sector* sector, const Vector2& normal);

    };
    
}
#endif
