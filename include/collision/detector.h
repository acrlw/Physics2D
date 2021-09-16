#ifndef PHYSICS2D_DETECTOR_H
#define PHYSICS2D_DETECTOR_H
#include "include/collision/algorithm/gjk.h"
#include "include/collision/algorithm/sat.h"
#include "include/collision/algorithm/mpr.h"
#include "include/math/math.h"
#include "include/geometry/shape.h"
#include "include/dynamics/body.h"

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
		static bool collide(Body* bodyA, Body* bodyB);
		static Collision detect(Body* bodyA, Body* bodyB);
		static std::optional<PointPair> distance(Body* bodyA, Body* bodyB);
		
	private:
	};
	
}
#endif
