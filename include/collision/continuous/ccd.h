#ifndef PHYSICS2D_COLLISION_CCD_H
#define PHYSICS2D_COLLISION_CCD_H
#include "include/collision/algorithm/gjk.h"
#include "include/collision/broadphase/dbvh.h"
#include "include/dynamics/body.h"
namespace Physics2D
{
	
	class CCD
	{
	public:
		struct AABBShot
		{
			AABBShot() = default;
			AABBShot(const AABB& box, const Body::PhysicsAttribute& attr, const real& t) : aabb(box), attribute(attr), time(t){}
			AABB aabb;
			Body::PhysicsAttribute attribute;
			real time = 0;
		};
		struct ShapeShot
		{
			ShapeShot() = default;
			Shape* shape = nullptr;
			Body::PhysicsAttribute attribute;
			real time = 0;
		};
		typedef std::vector<AABBShot> BroadphaseTrajectory;
		typedef std::vector<ShapeShot> NarrowphaseTrajectory;

		
		static std::tuple<bool, CCD::BroadphaseTrajectory> queryLeaf(DBVH::Node* root, Body* body, const real& dt);
		static std::tuple<std::vector<AABBShot>, AABB> buildTrajectoryAABB(Body* body, const real& dt);
		static std::optional<real> findBroadphaseRoot(Body* body1, Body* body2, const BroadphaseTrajectory& trajectory1, const BroadphaseTrajectory& trajectory2, const real& dt);
		static void findNarrowphaseRoot(Body* body1, Body* body2);
	private:
		
		static void queryNodes(DBVH::Node* node, const AABB& aabb, std::vector<DBVH::Node*>& nodes);
	};
}
#endif