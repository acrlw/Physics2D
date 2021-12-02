#include "../../../include/collision/algorithm/sat.h"

namespace Physics2D
{
	SATResult SAT::circleVsCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		//Convention: A is circle, B is capsule
		assert(shapeA.shape->type() == Shape::Type::Circle);
		assert(shapeB.shape->type() == Shape::Type::Capsule);
		SATResult result;
		return result;
	}

	SATResult SAT::circleVsSector(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Circle);
		assert(shapeB.shape->type() == Shape::Type::Sector);
		SATResult result;
		return result;
	}

	SATResult SAT::circleVsEdge(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		//Convention: A is circle, B is capsule
		assert(shapeA.shape->type() == Shape::Type::Circle);
		assert(shapeB.shape->type() == Shape::Type::Edge);

		SATResult result;
		Circle* circle = dynamic_cast<Circle*>(shapeA.shape);
		Edge* edge = dynamic_cast<Edge*>(shapeB.shape);

		auto* pointCircle = &result.contactPair[0].pointA;
		auto* pointEdge = &result.contactPair[0].pointB;
		
		Vector2 actualStart = shapeB.transform + edge->startPoint();
		Vector2 actualEnd = shapeB.transform + edge->endPoint();
		Vector2 normal = (actualStart - actualEnd).normal();

		if ((actualStart - shapeA.transform).dot(normal) < 0 &&
			(actualEnd - shapeB.transform).dot(normal) < 0)
			normal.negate();

		Vector2 projectedPoint = GeometryAlgorithm2D::pointToLineSegment(actualStart, actualEnd, shapeA.transform);
		Vector2 diff = projectedPoint - shapeA.transform;
		result.normal = diff.normal();
		real length = diff.length();
		result.isColliding = length < circle->radius();
		result.penetration = circle->radius() - length;
		*pointCircle = shapeA.transform + circle->radius() * result.normal;
		*pointEdge = projectedPoint;
		result.contactPairCount++;
		return result;
	}
	

	SATResult SAT::circleVsCircle(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Circle);
		assert(shapeB.shape->type() == Shape::Type::Circle);

		SATResult result;
		Circle* circleA = dynamic_cast<Circle*>(shapeA.shape);
		Circle* circleB = dynamic_cast<Circle*>(shapeB.shape);
		Vector2 ba = shapeA.transform - shapeB.transform;
		real dp = circleA->radius() + circleB->radius();
		real length = ba.length();
		if (length <= dp)
		{
			result.normal = ba.normal();
			result.penetration = dp - length;
			result.isColliding = true;
			result.contactPair[0].pointA = shapeA.transform - circleA->radius() * result.normal;
			result.contactPair[0].pointB = shapeB.transform + circleB->radius() * result.normal;
			result.contactPairCount++;
		}
		return result;
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

		ProjectedSegment segmentCircle = axisProjection(shapeA, circleA, normal);
		ProjectedSegment segmentPolygon = axisProjection(shapeB, polygonB, normal);
		
		auto [finalSegment, length] = ProjectedSegment::intersect(segmentCircle, segmentPolygon);
		
		if (length > 0)
			collidingAxis++;

		if (result.penetration > length)
		{
			result.penetration = length;
			result.normal = normal;
		}
		ProjectedPoint circlePoint, polygonPoint;
		circlePoint = segmentCircle.max == finalSegment.max ? finalSegment.max : finalSegment.min;
		polygonPoint = segmentPolygon.max == finalSegment.max ? finalSegment.max : finalSegment.min;
		
		ProjectedSegment segment;
		bool onPolygon = false;
		
		//polygon sat test
		for(int i = 0;i < polygonB->vertices().size() - 1;i++)
		{
			Vector2 v1 = shapeB.translate(polygonB->vertices()[i]);
			Vector2 v2 = shapeB.translate(polygonB->vertices()[i + 1]);
			Vector2 edge = v1 - v2;
			Vector2 normal = edge.perpendicular().normal();


			ProjectedSegment segmentC = axisProjection(shapeA, circleA, normal);
			ProjectedSegment segmentP = axisProjection(shapeB, polygonB, normal);

			auto [tempSegment, len] = ProjectedSegment::intersect(segmentC, segmentP);
			if (len > 0)
				collidingAxis++;

			if(result.penetration > len && len > 0)
			{
				result.penetration = len;
				result.normal = normal;
				segment = tempSegment;
				circlePoint = segmentC.max == tempSegment.max ? tempSegment.max : tempSegment.min;
			}
			
		}
		if (collidingAxis == polygonB->vertices().size())
			result.isColliding = true;
		
		result.contactPair[0].pointA = circlePoint.vertex;
		result.contactPair[0].pointB = circlePoint.vertex + -result.normal * result.penetration;
		result.contactPairCount++;


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
			
			Vector2 finalNormal;
			real minLength = Constant::Max;
			int collidingAxis = 0;
			ProjectedSegment segment;

			ProjectedPoint targetAPoint, targetBPoint;
			for(int i = 0;i < polyA->vertices().size() - 1;i++)
			{
				Vector2 v1 = polygonA.translate(polyA->vertices()[i]);
				Vector2 v2 = polygonA.translate(polyA->vertices()[i + 1]);
				Vector2 edge = v1 - v2;
				Vector2 normal = edge.perpendicular().normal();

				ProjectedSegment segmentA = axisProjection(polygonA, polyA, normal);
				ProjectedSegment segmentB = axisProjection(polygonB, polyB, normal);

				auto [finalSegment, length] = ProjectedSegment::intersect(segmentA, segmentB);
				if (length > 0)
					collidingAxis++;

				ProjectedPoint polyAPoint, polyBPoint;
				polyAPoint = segmentA.max == finalSegment.max ? finalSegment.max : finalSegment.min;
				polyBPoint = segmentB.max == finalSegment.max ? finalSegment.max : finalSegment.min;
				

				if(minLength > length)
				{
					minLength = length;
					finalNormal = normal;
					targetAPoint = polyAPoint;
					targetBPoint = polyBPoint;
				}
			}

			return std::make_tuple(finalNormal, minLength, collidingAxis, targetAPoint, targetBPoint);
		};

		auto [normal1, length1, axis1, polyAPoint1, polyBPoint1] = test(shapeA, shapeB);
		auto [normal2, length2, axis2, polyBPoint2, polyAPoint2] = test(shapeB, shapeA);
		if ( axis1 + axis2 == polyA->vertices().size() + polyB->vertices().size() - 2 )
			result.isColliding = true;

		ProjectedPoint* pointA;
		ProjectedPoint* pointB;

		if(length1 < length2)
		{
			result.penetration = length1;
			result.normal = normal1;
			pointA = &polyAPoint1;
			pointB = &polyBPoint1;
			//do clipping
		}
		else
		{
			result.penetration = length2;
			result.normal = normal2;
			pointA = &polyAPoint2;
			pointB = &polyBPoint2;
			//do clipping
		}
		//auto clipEdgeA = ContactGenerator::findClipEdge(polyA->vertices(), pointA->index, result.normal);
		//auto clipEdgeB = ContactGenerator::findClipEdge(polyB->vertices(), pointB->index, -result.normal);
		//auto pairList = ContactGenerator::clip(clipEdgeA, clipEdgeB, result.normal);

		return result;
	}

	SATResult SAT::polygonVsCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Polygon);
		assert(shapeB.shape->type() == Shape::Type::Capsule);
		return SATResult();
	}

	SATResult SAT::polygonVsSector(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Polygon);
		assert(shapeB.shape->type() == Shape::Type::Sector);
		return SATResult();
	}
	SATResult SAT::capsuleVsEdge(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Capsule);
		assert(shapeB.shape->type() == Shape::Type::Edge);
		return SATResult();
	}
	SATResult SAT::capsuleVsCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Capsule);
		assert(shapeB.shape->type() == Shape::Type::Capsule);
		return SATResult();
	}
	SATResult SAT::capsuleVsSector(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Capsule);
		assert(shapeB.shape->type() == Shape::Type::Sector);
		return SATResult();
	}
	SATResult SAT::sectorVsSector(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Sector);
		assert(shapeB.shape->type() == Shape::Type::Sector);
		return SATResult();
	}
	SATResult SAT::polygonVsEdge(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape->type() == Shape::Type::Polygon);
		assert(shapeB.shape->type() == Shape::Type::Edge);
		return SATResult();
	}
	

	ProjectedSegment SAT::axisProjection(const ShapePrimitive& shape, Polygon* polygon, const Vector2& normal)
	{
		ProjectedPoint minPoint, maxPoint;
		minPoint.value = Constant::Max;
		maxPoint.value = Constant::NegativeMin;

		for(size_t i = 0;i < polygon->vertices().size();i++)
		{
			Vector2 vertex = shape.translate(polygon->vertices()[i]);
			real value = vertex.dot(normal);

			if (value < minPoint.value)
			{
				minPoint.vertex = vertex;
				minPoint.value = value;
				minPoint.index = i;
			}

			if (value > maxPoint.value)
			{
				maxPoint.vertex = vertex;
				maxPoint.value = value;
				maxPoint.index = i;
			}
		}

		ProjectedSegment segment;
		segment.max = maxPoint;
		segment.min = minPoint;
		return segment;
	}

	ProjectedSegment SAT::axisProjection(const ShapePrimitive& shape, Circle* circle, const Vector2& normal)
	{
		ProjectedPoint minCircle, maxCircle;

		maxCircle.vertex = shape.transform + normal * circle->radius();
		maxCircle.value = shape.transform.dot(normal) + circle->radius();

		minCircle.vertex = shape.transform - normal * circle->radius();
		minCircle.value = shape.transform.dot(normal) - circle->radius();

		ProjectedSegment segmentCircle;

		segmentCircle.min = minCircle;
		segmentCircle.max = maxCircle;

		return segmentCircle;
	}

	ProjectedSegment SAT::axisProjection(const ShapePrimitive& shape, Ellipse* ellipse, const Vector2& normal)
	{
		ProjectedPoint minEllipse, maxEllipse;
		Vector2 rot_dir = Matrix2x2(-shape.rotation).multiply(normal);
		maxEllipse.vertex = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), rot_dir);
		maxEllipse.vertex = shape.translate(maxEllipse.vertex);
		maxEllipse.value = maxEllipse.vertex.dot(normal);
		
		rot_dir = Matrix2x2(-shape.rotation).multiply(-normal);
		minEllipse.vertex = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), rot_dir);
		minEllipse.vertex = shape.translate(minEllipse.vertex);
		minEllipse.value = minEllipse.vertex.dot(normal);
		
		ProjectedSegment segmentEllipse;

		segmentEllipse.min = minEllipse;
		segmentEllipse.max = maxEllipse;

		return segmentEllipse;
		 
	}

	ProjectedSegment SAT::axisProjection(const ShapePrimitive& shape, Capsule* capsule, const Vector2& normal)
	{
		ProjectedPoint min, max;
		Vector2 direction = Matrix2x2(-shape.rotation).multiply(normal);
		Vector2 p1 = GeometryAlgorithm2D::calculateCapsuleProjectionPoint(capsule->width(), capsule->height(), direction);
		Vector2 p2 = GeometryAlgorithm2D::calculateCapsuleProjectionPoint(capsule->width(), capsule->height(), -direction);
		p1 = shape.translate(p1);
		p2 = shape.translate(p2);

		max.vertex = p1;
		max.value = max.vertex.dot(normal);
		min.vertex = p2;
		min.value = min.vertex.dot(normal);

		ProjectedSegment segment;

		segment.min = min;
		segment.max = max;

		return segment;
	}

	ProjectedSegment SAT::axisProjection(const ShapePrimitive& shape, Sector* sector, const Vector2& normal)
	{
		ProjectedPoint min, max;
		Vector2 direction = Matrix2x2(-shape.rotation).multiply(normal);
		Vector2 p1 = GeometryAlgorithm2D::calculateSectorProjectionPoint(sector->startRadian(), sector->spanRadian(), sector->radius(), direction);
		Vector2 p2 = GeometryAlgorithm2D::calculateSectorProjectionPoint(sector->startRadian(), sector->spanRadian(), sector->radius(), -direction);
		p1 = shape.translate(p1);
		p2 = shape.translate(p2);

		max.vertex = p1;
		max.value = max.vertex.dot(normal);
		min.vertex = p2;
		min.value = min.vertex.dot(normal);

		ProjectedSegment segment;

		segment.min = min;
		segment.max = max;

		return segment;
	}
	
	std::tuple<ProjectedSegment, real> ProjectedSegment::intersect(const ProjectedSegment& s1, const ProjectedSegment& s2)
	{
		real difference = Constant::NegativeMin;
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
		else if(s1.min.value >= s2.min.value && s1.max.value <= s2.max.value)
		{
			if((s1.max.value - s2.min.value) > (s2.max.value - s1.min.value))
			{
				difference = s1.max.value - s2.min.value;
				result.max = s1.max;
				result.min = s2.min;
			}
			else
			{
				difference = s2.max.value - s1.min.value;
				result.max = s2.max;
				result.min = s1.min;
			}
		}
		else if (s2.min.value >= s1.min.value && s2.max.value <= s1.max.value)
		{
			if ((s2.max.value - s1.min.value) > (s1.max.value - s2.min.value))
			{
				difference = s2.max.value - s1.min.value;
				result.max = s2.max;
				result.min = s1.min;
			}
			else
			{
				difference = s1.max.value - s2.min.value;
				result.max = s1.max;
				result.min = s2.min;
			}
		}
		return std::make_tuple(result, difference);
	}
	bool ProjectedPoint::operator==(const ProjectedPoint& rhs)
	{
		return vertex.fuzzyEqual(rhs.vertex) && value == rhs.value;
	}
}
