#ifndef PHYSICS2D_DETECTOR_H
#define PHYSICS2D_DETECTOR_H

#include "physics2d_math.h"
#include "physics2d_shape.h"
#include "physics2d_body.h"
#include "physics2d_narrowphase.h"

namespace Physics2D
{
	struct PHYSICS2D_API Collision
	{
		bool isColliding = false;
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		ContactPair contactList;
		Vector2 normal;
		real penetration = 0;
	};

	class PHYSICS2D_API Detector
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

		static CollisionInfo distance(Body* bodyA, Body* bodyB);
		static CollisionInfo distance(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
		static CollisionInfo distance(Body* bodyA, const ShapePrimitive& shapeB);
		static CollisionInfo distance(const ShapePrimitive& shapeA, Body* bodyB);

	private:
	};
}
#endif
