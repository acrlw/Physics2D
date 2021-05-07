#include "include/collision/algorithm/sat.h"

namespace Physics2D
{
	std::optional<PenetrationInfo> SAT::circleVsCircle(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Circle);
		assert(shapeB.shape->type() == Shape::Type::Circle);

		Circle* circleA = dynamic_cast<Circle*>(shapeA.shape);
		Circle* circleB = dynamic_cast<Circle*>(shapeB.shape);
		Vector2 ba = shapeA.transform - shapeB.transform;
		real dp = circleA->radius() + circleB->radius();
		real length = ba.length();
		if (length <= dp)
		{
			PenetrationInfo info;
			info.normal = ba.normal();
			info.penetration = dp - length;
			return std::optional<PenetrationInfo>(info);
		}
		return std::nullopt;
	}

	SATResult SAT::circleVsPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Circle);
		assert(shapeB.shape->type() == Shape::Type::Polygon);

		Circle* circleA = dynamic_cast<Circle*>(shapeA.shape);
		Polygon* polygonB = dynamic_cast<Polygon*>(shapeB.shape);

		uint16_t collidingAxis = 0;
		SATResult result;
		
		//circle sat test

		//finding circle axis
		real minLength = Constant::Max;
		Vector2 closest;
		for (auto& elem : polygonB->vertices())
		{
			Vector2 vertex = shapeB.translate(elem);
			real length = (vertex - shapeA.transform).lengthSquare();
			if (minLength > length)
			{
				minLength = length;
				closest = vertex;
			}
		}
		Vector2 normal = closest.normal();

		ProjectedPoint minCircle, maxCircle;
		
		maxCircle.vertex = normal * circleA->radius();
		maxCircle.value = circleA->radius();
		
		minCircle.vertex = -normal * circleA->radius();
		maxCircle.value = -circleA->radius();

		ProjectedSegment segmentCircle;
		
		segmentCircle.min = minCircle;
		segmentCircle.max = maxCircle;

		ProjectedSegment segmentPolygon = axisProjection(shapeB, polygonB, normal);
		
		auto [finalSegment, length] = ProjectedSegment::intersect(segmentCircle, segmentPolygon);
		
		if (length < 0)
			collidingAxis++;

		if (result.penetration < length)
		{
			result.penetration = length;
			result.normal = normal;
		}

		ProjectedEdge target;
		ProjectedSegment segment;
		bool onPolygon = false;
		
		//polygon sat test
		for(int i = 0;i < polygonB->vertices().size() - 2;i++)
		{
			Vector2 v1 = shapeB.translate(polygonB->vertices()[i]);
			Vector2 v2 = shapeB.translate(polygonB->vertices()[i + 1]);
			Vector2 edge = v1 - v2;
			Vector2 normal = edge.perpendicular().normal();


			ProjectedPoint minC, maxC;
			maxC.vertex = normal * circleA->radius();
			maxC.value = circleA->radius();

			minC.vertex = -normal * circleA->radius();
			minC.value = -circleA->radius();

			ProjectedSegment segmentC;

			segmentC.min = minC;
			segmentC.max = maxC;

			ProjectedSegment segmentP = axisProjection(shapeB, polygonB, normal);

			auto [tempSegment, len] = ProjectedSegment::intersect(segmentC, segmentP);
			if (len < 0)
				collidingAxis++;

			if(result.penetration < length)
			{
				target.vertex1 = v1;
				target.vertex2 = v2;
				result.penetration = length;
				result.normal = normal;
				segment = tempSegment;
				onPolygon = true;
			}
			
		}
		if (collidingAxis == polygonB->vertices().size())
			result.isColliding = true;

		result.targetEdge = target;
		
		result.pointPair = onPolygon ? segment : finalSegment;


		return result;
	}

	SATResult SAT::polygonVsPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Polygon);
		assert(shapeB.shape->type() == Shape::Type::Polygon);

		Polygon* polyA = dynamic_cast<Polygon*>(shapeA.shape);
		Polygon* polyB = dynamic_cast<Polygon*>(shapeB.shape);
		
		SATResult result;

		auto test = [](const ShapePrimitive& polygonA, const ShapePrimitive& polygonB)
		{
			Polygon* polyA = dynamic_cast<Polygon*>(polygonA.shape);
			Polygon* polyB = dynamic_cast<Polygon*>(polygonB.shape);
			
			ProjectedEdge targetEdge;
			Vector2 finalNormal;
			real minLength = Constant::Max;
			int collidingAxis = 0;
			ProjectedSegment segment;
			
			for(int i = 0;i < polyA->vertices().size() - 2;i++)
			{
				Vector2 v1 = polygonA.translate(polyA->vertices()[i]);
				Vector2 v2 = polygonA.translate(polyA->vertices()[i + 1]);
				Vector2 edge = v1 - v2;
				Vector2 normal = edge.perpendicular().normal();

				ProjectedSegment segmentA = axisProjection(polygonA, polyA, normal);
				ProjectedSegment segmentB = axisProjection(polygonB, polyB, normal);

				auto [finalSegment, length] = ProjectedSegment::intersect(segmentA, segmentB);
				if (length < 0)
					collidingAxis++;

				if(minLength > length)
				{
					minLength = length;
					targetEdge.vertex1 = v1;
					targetEdge.vertex2 = v2;
					finalNormal = normal;
					segment = finalSegment;
				}
			}

			return std::make_tuple(targetEdge, finalNormal, minLength, collidingAxis, segment);
		};

		auto [edge1, normal1, length1, axis1, segment1] = test(shapeA, shapeB);
		auto [edge2, normal2, length2, axis2, segment2] = test(shapeA, shapeB);
		if ((axis1 + axis2) == polyA->vertices().size() + polyB->vertices().size() - 2)
			result.isColliding = true;

		if(length1 < length2)
		{
			result.targetEdge = edge1;
			result.penetration = length1;
			result.normal = normal1;
			result.pointPair = segment1;
		}
		else
		{
			result.targetEdge = edge2;
			result.penetration = length2;
			result.normal = normal2;
			result.pointPair = segment2;
		}

		return result;
	}

	ProjectedSegment SAT::axisProjection(const ShapePrimitive& shape, Polygon* polygon, const Vector2& normal)
	{
		ProjectedPoint minPoint, maxPoint;
		minPoint.value = Constant::Max;
		maxPoint.value = Constant::Min;

		for (const Vector2& elem : polygon->vertices())
		{
			Vector2 vertex = shape.translate(elem);
			real value = vertex.dot(normal);

			if (value < minPoint.value)
			{
				minPoint.vertex = vertex;
				minPoint.value = value;
			}

			if (value > maxPoint.value)
			{
				maxPoint.vertex = vertex;
				maxPoint.value = value;
			}
		}

		ProjectedSegment segment;
		segment.max = maxPoint;
		segment.min = minPoint;

		return segment;
	}
	
	std::tuple<ProjectedSegment, real> ProjectedSegment::intersect(const ProjectedSegment& s1, const ProjectedSegment& s2)
	{
		real difference = 0;
		ProjectedSegment result;
		if(s1.min.value <= s2.min.value && s1.max.value <= s2.max.value)
		{
			difference = s1.max.value - s2.min.value;
			result.max = s1.max;
			result.min = s2.min;
		}
		else if (s2.min.value <= s1.min.value && s2.max.value <= s1.max.value)
		{
			difference = s2.max.value - s1.min.value;
			result.max = s1.min;
			result.min = s2.max;
		}
		return std::make_tuple(result, difference);
	}
}
