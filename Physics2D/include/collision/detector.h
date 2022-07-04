#ifndef PHYSICS2D_DETECTOR_H
#define PHYSICS2D_DETECTOR_H
#include "../collision/algorithm/gjk.h"
#include "../collision/algorithm/sat.h"
#include "../collision/algorithm/mpr.h"
#include "../math/math.h"
#include "../geometry/shape.h"
#include "../dynamics/body.h"

namespace Physics2D
{
	struct Collision
	{
		bool isColliding = false;
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		std::vector<PointPair> contactList;
		Vector2 normal;
		real penetration = 0;
	};

	class Detector
	{

	public:
		static bool collide(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
		static bool collide(Body* bodyA, Body* bodyB);
		static bool collide(const ShapePrimitive& shapeA, Body* bodyB);
		static bool collide(Body* bodyA, const ShapePrimitive& shapeB);

		static Collision detect(Body* bodyA, Body* bodyB);
		static Collision detect(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
		static Collision detect(Body* bodyA, const ShapePrimitive& shapeB);
		static Collision detect(const ShapePrimitive& shapeA, Body* bodyB);

		static PointPair distance(Body* bodyA, Body* bodyB);
		static PointPair distance(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
		static PointPair distance(Body* bodyA, const ShapePrimitive& shapeB);
		static PointPair distance(const ShapePrimitive& shapeA, Body* bodyB);
		
	private:
	};
	
}
#endif
