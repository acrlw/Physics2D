#ifndef PHYSICS2D_COLLISION_CCD_H
#define PHYSICS2D_COLLISION_CCD_H
#include "include/collision/detector.h"
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
		struct IndexSection
		{
			int forward = -1;
			int backward = -1;
		};
		typedef std::vector<AABBShot> BroadphaseTrajectory;

		
		static std::tuple<bool, CCD::BroadphaseTrajectory> queryLeaf(DBVH::Node* root, Body* body, const real& dt);
		static std::tuple<std::vector<AABBShot>, AABB> buildTrajectoryAABB(Body* body, const Vector2& target, const real& dt);
		static std::optional<IndexSection> findBroadphaseRoot(Body* body1, const BroadphaseTrajectory& trajectory1, Body* body2, const BroadphaseTrajectory& trajectory2, const real& dt);
		static std::optional<real> findNarrowphaseRoot(Body* body1, const BroadphaseTrajectory& trajectory1, Body* body2, const BroadphaseTrajectory& trajectory2, const IndexSection& index, const real& dt);
	private:
		
		static void queryNodes(DBVH::Node* node, const AABB& aabb, std::vector<DBVH::Node*>& nodes);
	};
}
#endif