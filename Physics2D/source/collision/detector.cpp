#include "../../include/collision/detector.h"
namespace Physics2D
{
	
	bool Detector::collide(Body* bodyA, Body* bodyB)
	{
		assert(bodyA != nullptr && bodyB != nullptr);

		ShapePrimitive shapeA, shapeB;
		shapeA.shape = bodyA->shape();
		shapeA.rotation = bodyA->rotation();
		shapeA.transform = bodyA->position();

		shapeB.shape = bodyB->shape();
		shapeB.rotation = bodyB->rotation();
		shapeB.transform = bodyB->position();

		auto [isColliding, simplex] = GJK::gjk(shapeA, shapeB);

		if (shapeA.transform.fuzzyEqual(shapeB.transform) && !isColliding)
			isColliding = simplex.containOrigin(true);

		return isColliding;
	}
	Collision Detector::detect(Body* bodyA, Body* bodyB)
	{
		Collision result;

		if (bodyA == nullptr || bodyB == nullptr)
			return result;

		if (bodyA == bodyB)
			return result;

		if (bodyA->id() > bodyB->id())
		{
			Body* temp = bodyA;
			bodyA = bodyB;
			bodyB = temp;
		}

		result.bodyA = bodyA;
		result.bodyB = bodyB;

		ShapePrimitive shapeA, shapeB;
		shapeA.shape = bodyA->shape();
		shapeA.rotation = bodyA->rotation();
		shapeA.transform = bodyA->position();

		shapeB.shape = bodyB->shape();
		shapeB.rotation = bodyB->rotation();
		shapeB.transform = bodyB->position();

		auto [isColliding, simplex] = GJK::gjk(shapeA, shapeB);

		if (shapeA.transform.fuzzyEqual(shapeB.transform) && !isColliding)
			isColliding = simplex.containOrigin(true);

		result.isColliding = isColliding;

		
		if (isColliding)
		{
			auto oldSimplex = simplex;
			simplex = GJK::epa(shapeA, shapeB, simplex);
			PenetrationSource source = GJK::dumpSource(simplex);
			
			const auto info = GJK::dumpInfo(source);
			result.normal = info.normal;
			result.penetration = info.penetration;

			auto [clipEdgeA, clipEdgeB] = ContactGenerator::recognize(shapeA, shapeB, info.normal);
			auto pairList = ContactGenerator::clip(clipEdgeA, clipEdgeB, info.normal);

			bool pass = false;
			for(auto& elem: pairList)
				if(realEqual((elem.pointA - elem.pointB).lengthSquare(), result.penetration * result.penetration))
					pass = true;

			//if fail, there must be a deeper contact point, use it:
			if(pass)
				result.contactList = pairList;
			else 
				result.contactList.emplace_back(GJK::dumpPoints(source));
		}
		assert(result.contactList.size() != 3);
		return result;
	}

	PointPair Detector::distance(Body* bodyA, Body* bodyB)
	{
		PointPair result;
		if (bodyA == nullptr || bodyB == nullptr)
			return result;

		if (bodyA == bodyB)
			return result;

		ShapePrimitive shapeA, shapeB;
		shapeA.shape = bodyA->shape();
		shapeA.rotation = bodyA->rotation();
		shapeA.transform = bodyA->position();

		shapeB.shape = bodyB->shape();
		shapeB.rotation = bodyB->rotation();
		shapeB.transform = bodyB->position();

		return GJK::distance(shapeA, shapeB);
	}
}