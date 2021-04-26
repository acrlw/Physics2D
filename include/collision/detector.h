#ifndef PHYSICS2D_DETECTOR_H
#define PHYSICS2D_DETECTOR_H
#include "include/collision/algorithm/gjk.h"
#include "include/collision/algorithm/sat.h"
#include "include/collision/contact.h"
#include "include/math/math.h"
#include "include/geometry/shape.h"
#include "include/dynamics/body.h"

namespace Physics2D
{
	struct Collision
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		std::vector<PointPair> contactList;
		Vector2 normal;
		real penetration = 0;
	};

	class Detector
	{
	public:
		static Collision detect(Body* bodyA, Body* bodyB)
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

			auto [isColliding, simplex] = GJK::gjk(shapeA, shapeB);
			if (isColliding)
			{
				result.bodyA = bodyA;
				result.bodyB = bodyB;

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
					auto temp = ContactGenerator::generate(shapeB, edge, source.b1, info);
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
					auto temp = ContactGenerator::generate(shapeB, edge, source.a1, info, true);
					if (temp.has_value())
						result.contactList = temp.value();
					else
						result.contactList.emplace_back(GJK::dumpContacts(source, info));
				}
				else
				{
					auto temp = ContactGenerator::clip({source.a1, source.a2}, {source.b1, source.b2});
					if (temp.has_value())
						result.contactList = temp.value();
					else
						result.contactList.emplace_back(GJK::dumpContacts(source, info));
				}
			}

			return result;
		}

		static std::optional<PointPair> distance(Body* bodyA, Body* bodyB)
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

	private:
	};
}
#endif
