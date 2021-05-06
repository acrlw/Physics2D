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

	std::optional<Vector2> SAT::circleVsPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Circle);
		assert(shapeB.shape->type() == Shape::Type::Polygon);

		Circle* circleA = dynamic_cast<Circle*>(shapeA.shape);
		Polygon* polygonB = dynamic_cast<Polygon*>(shapeB.shape);

		int collidingAxis = 0;
		real minDistance = Constant::Max;
		
		Vector2 dp = shapeB.transform - shapeA.transform;

		auto transform = [shapeA, shapeB, dp](const Vector2& vertex)
		{
			return Matrix2x2(shapeB.rotation).multiply(vertex) + dp;
		};
		
		//circle sat test
		real minLength = Constant::Max;
		Vector2 closest;
		for (auto& elem : polygonB->vertices())
		{
			Vector2 vertex = transform(elem);
			real length = vertex.lengthSquare();
			if (minLength > length)
			{
				minLength = length;
				closest = vertex;
			}
		}
		Vector2 normal = closest.normal();

		ProjectedPoint minPolygon, maxPolygon;
		minPolygon.value = Constant::Max;
		maxPolygon.value = Constant::Min;
		
		for (auto& elem : polygonB->vertices())
		{
			Vector2 vertex = transform(elem);
			real value = vertex.dot(normal);
			
			if(value < minPolygon.value)
			{
				minPolygon.vertex = vertex;
				minPolygon.value = value;
			}
			
			if (value > maxPolygon.value)
			{
				maxPolygon.vertex = vertex;
				maxPolygon.value = value;
			}
		}

		ProjectedPoint minCircle, maxCircle;
		
		maxCircle.vertex = normal * circleA->radius();
		maxCircle.value = circleA->radius();
		
		minCircle.vertex = -normal * circleA->radius();
		maxCircle.value = -circleA->radius();

		ProjectedSegment segmentCircle, segmentPolygon;
		
		segmentCircle.min = minCircle;
		segmentCircle.max = maxCircle;
		
		segmentPolygon.min = minPolygon;
		segmentPolygon.max = maxPolygon;

		auto [finalSegment, length] = ProjectedSegment::intersect(segmentCircle, segmentPolygon);
		
		if (length < 0)
			collidingAxis++;

		if (minDistance < length)
			minDistance = length;

		//polygon sat test
		for(int i = 0;i < polygonB->vertices().size() - 2;i++)
		{
			Vector2 v1 = transform(polygonB->vertices()[i]);
			Vector2 v2 = transform(polygonB->vertices()[i + 1]);
			Vector2 edge = polygonB->vertices()[i] - polygonB->vertices()[i + 1];
			Vector2 normal = edge.normal();
			
			
		}
		
		return std::nullopt;
	}

	std::optional<Vector2> SAT::polygonVsPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Polygon);
		assert(shapeB.shape->type() == Shape::Type::Polygon);

		Polygon* polygonA = dynamic_cast<Polygon*>(shapeA.shape);
		Polygon* polygonB = dynamic_cast<Polygon*>(shapeB.shape);
		Vector2 dp = shapeB.transform - shapeA.transform;
		return std::nullopt;
	}
	
	std::tuple<SAT::ProjectedSegment, real> SAT::ProjectedSegment::intersect(const ProjectedSegment& s1, const ProjectedSegment& s2)
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
			result.max = s2.max;
			result.min = s1.min;
		}
		return std::make_tuple(result, difference);
	}
}
