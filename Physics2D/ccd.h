#ifndef PHYSICS2D_COLLISION_CCD_H
#define PHYSICS2D_COLLISION_CCD_H
#include "detector.h"
#include "dbvh.h"
#include "body.h"
#include "tree.h"
namespace Physics2D
{
	/// <summary>
	/// Continuous Collision Detection
	///	This class is implemented by bisection and re-sampling. Both them are costly.
	/// </summary>
	class CCD
	{
	public:
		struct AABBShot
		{
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
		struct CCDPair
		{
			CCDPair() = default;
			CCDPair(const real& time, Body* target) : toi(time), body(target){}
			real toi = 0.0;
			Body* body = nullptr;
		};
		typedef Container::Vector<AABBShot> BroadphaseTrajectory;
		static std::tuple<BroadphaseTrajectory, AABB> buildTrajectoryAABB(Body* body, const real& dt);
		static std::tuple<Container::Vector<AABBShot>, AABB> buildTrajectoryAABB(Body* body, const Vector2& target, const real& dt);
		static std::optional<IndexSection> findBroadphaseRoot(Body* staticBody, const BroadphaseTrajectory& staticTrajectory, Body* dynamicBody, const BroadphaseTrajectory& dynamicTrajectory, const real& dt);
		static std::optional<real> findNarrowphaseRoot(Body* staticBody, const BroadphaseTrajectory& staticTrajectory, Body* dynamicBody, const BroadphaseTrajectory& dynamicTrajectory, const IndexSection& index, const real& dt);
		static std::optional<Container::Vector<CCDPair>> query(DBVH::Node* root, Body* body, const real& dt);
        static std::optional<Container::Vector<CCDPair>> query(Tree& tree, Body* body, const real& dt);
        static std::optional<real> earliestTOI(const Container::Vector<CCDPair>& pairs, const real& epsilon = Constant::GeometryEpsilon);
	};
}
#endif