#include "include/collision/detector.h"

namespace Physics2D
{
	Collision Detector::detect(Body* bodyA, Body* bodyB)
	{
		Collision result;

		if (bodyA == nullptr || bodyB == nullptr)
			return result;

		if (bodyA == bodyB)
			return result;

		result.bodyA = bodyA;
		result.bodyB = bodyB;

		ShapePrimitive shapeA, shapeB;
		shapeA.shape = bodyA->shape();
		shapeA.rotation = bodyA->angle();
		shapeA.transform = bodyA->position();

		shapeB.shape = bodyB->shape();
		shapeB.rotation = bodyB->angle();
		shapeB.transform = bodyB->position();

		AABB a = AABB::fromShape(shapeA);
		AABB b = AABB::fromShape(shapeB);
		if (!a.collide(b))
			return result;


		
		//auto [direction, simplex] = MPR::discover(shapeA, shapeB);
		//auto [isColliding, portal] = MPR::refine(shapeA, shapeB, simplex, direction);
		//if (shapeA.transform.fuzzyEqual(shapeB.transform) && !isColliding)
		//	isColliding = simplex.containOrigin(true);

		//result.isColliding = isColliding;
		
		auto [isColliding, simplex] = GJK::gjk(shapeA, shapeB);
		
		if(shapeA.transform.fuzzyEqual(shapeB.transform) && !isColliding)
			isColliding = simplex.containOrigin(true);
		
		result.isColliding = isColliding;
		
		if (isColliding)
		{

			//portal.vertices.erase(portal.vertices.begin());
			//PenetrationSource source = GJK::dumpSource(portal);
			
			simplex = GJK::epa(shapeA, shapeB, simplex);
			PenetrationSource source = GJK::dumpSource(simplex);
			

			const auto info = GJK::dumpInfo(source);
			result.normal = info.normal;
			result.penetration = info.penetration;

			if (source.a1 == source.a2 && source.b1 == source.b2)
			{
				result.contactList.emplace_back(GJK::dumpContacts(source, info));
			}
			else if (source.a1 != source.a2 && source.b1 == source.b2)
			{
				ContactEdge edge;
				edge.point1 = source.a1;
				edge.point2 = source.a2;
				auto temp = ContactGenerator::generate(shapeB, edge, source.b1, info, true);
				if (temp.has_value())
					result.contactList = temp.value();
				else
					result.contactList.emplace_back(GJK::dumpContacts(source, info));
			}
			else if (source.a1 == source.a2 && source.b1 != source.b2)
			{
				ContactEdge edge;
				edge.point1 = source.b1;
				edge.point2 = source.b2;
				auto temp = ContactGenerator::generate(shapeA, edge, source.a1, info, false);
				if (temp.has_value())
					result.contactList = temp.value();
				else
					result.contactList.emplace_back(GJK::dumpContacts(source, info));
			}
			else
			{
				auto temp = ContactGenerator::clip({ source.a1, source.a2 }, { source.b1, source.b2 });
				if (temp.has_value())
					result.contactList = temp.value();
				else
					result.contactList.emplace_back(GJK::dumpContacts(source, info));
			}
		}
		assert(result.contactList.size() != 3);
		return result;
	}

	std::optional<PointPair> Detector::distance(Body* bodyA, Body* bodyB)
	{
		if (bodyA == nullptr || bodyB == nullptr)
			return std::nullopt;

		if (bodyA == bodyB)
			return std::nullopt;

		ShapePrimitive shapeA, shapeB;
		shapeA.shape = bodyA->shape();
		shapeA.rotation = bodyA->angle();
		shapeA.transform = bodyA->position();

		shapeB.shape = bodyB->shape();
		shapeB.rotation = bodyB->angle();
		shapeB.transform = bodyB->position();

		return std::optional<PointPair>(GJK::distance(shapeA, shapeB));
	}
}