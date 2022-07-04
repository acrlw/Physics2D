#include "../../include/collision/detector.h"
namespace Physics2D
{

	bool Detector::collide(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		auto [isColliding, simplex] = GJK::gjk(shapeA, shapeB);

		if (shapeA.transform.fuzzyEqual(shapeB.transform) && !isColliding)
			isColliding = simplex.containOrigin(true);

		return isColliding;
	}
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

		return collide(shapeA, shapeB);
	}
	bool Detector::collide(const ShapePrimitive& shapeA, Body* bodyB)
	{
		assert(shapeA.shape != nullptr && bodyB != nullptr);

		ShapePrimitive shapeB;
		shapeB.shape = bodyB->shape();
		shapeB.rotation = bodyB->rotation();
		shapeB.transform = bodyB->position();

		return collide(shapeA, shapeB);
	}
	bool Detector::collide(Body* bodyA, const ShapePrimitive& shapeB)
	{
		assert(shapeB.shape != nullptr && bodyA != nullptr);

		ShapePrimitive shapeA;
		shapeA.shape = bodyA->shape();
		shapeA.rotation = bodyA->rotation();
		shapeA.transform = bodyA->position();

		return collide(shapeA, shapeB);
	}
	Collision Detector::detect(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		Collision result;
		assert(shapeA.shape != nullptr && shapeB.shape != nullptr);

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
			for (auto& elem : pairList)
				if (realEqual((elem.pointA - elem.pointB).lengthSquare(), result.penetration * result.penetration))
					pass = true;

			//if fail, there must be a deeper contact point, use it:
			if (pass)
				result.contactList = pairList;
			else
				result.contactList.emplace_back(GJK::dumpPoints(source));
		}
		assert(result.contactList.size() != 3);
		return result;
	}
	Collision Detector::detect(Body* bodyA, const ShapePrimitive& shapeB)
	{
		Collision result;

		assert(bodyA != nullptr && shapeB.shape != nullptr);

		ShapePrimitive shapeA;
		shapeA.shape = bodyA->shape();
		shapeA.rotation = bodyA->rotation();
		shapeA.transform = bodyA->position();


		result = detect(shapeA, shapeB);
		result.bodyA = bodyA;
		result.bodyB = nullptr;

		return result;

	}
	Collision Detector::detect(const ShapePrimitive& shapeA, Body* bodyB)
	{
		Collision result;

		assert(shapeA.shape != nullptr && bodyB != nullptr);

		ShapePrimitive shapeB;
		shapeB.shape = bodyB->shape();
		shapeB.rotation = bodyB->rotation();
		shapeB.transform = bodyB->position();

		result = detect(shapeA, shapeB);
		result.bodyA = nullptr;
		result.bodyB = bodyB;

		return result;
	}
	Collision Detector::detect(Body* bodyA, Body* bodyB)
	{
		Collision result;

		assert(bodyA != nullptr && bodyB != nullptr);

		if (bodyA == bodyB)
			return result;

		if (bodyA->id() > bodyB->id())
		{
			Body* temp = bodyA;
			bodyA = bodyB;
			bodyB = temp;
		}


		ShapePrimitive shapeA, shapeB;
		shapeA.shape = bodyA->shape();
		shapeA.rotation = bodyA->rotation();
		shapeA.transform = bodyA->position();

		shapeB.shape = bodyB->shape();
		shapeB.rotation = bodyB->rotation();
		shapeB.transform = bodyB->position();

		result = detect(shapeA, shapeB);
		result.bodyA = bodyA;
		result.bodyB = bodyB;

		return result;
	}
	PointPair Detector::distance(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape != nullptr && shapeB.shape != nullptr);
		return GJK::distance(shapeA, shapeB);
	}
	PointPair Detector::distance(Body* bodyA, const ShapePrimitive& shapeB)
	{
		assert(bodyA != nullptr && shapeB.shape != nullptr);

		ShapePrimitive shapeA;
		shapeA.shape = bodyA->shape();
		shapeA.rotation = bodyA->rotation();
		shapeA.transform = bodyA->position();

		return GJK::distance(shapeA, shapeB);
	}
	PointPair Detector::distance(const ShapePrimitive& shapeA, Body* bodyB)
	{
		assert(bodyB != nullptr && shapeA.shape != nullptr);

		ShapePrimitive shapeB;
		shapeB.shape = bodyB->shape();
		shapeB.rotation = bodyB->rotation();
		shapeB.transform = bodyB->position();

		return GJK::distance(shapeA, shapeB);
	}
	PointPair Detector::distance(Body* bodyA, Body* bodyB)
	{
		PointPair result;
		assert(bodyA != nullptr && bodyB != nullptr);

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