#ifndef PHYSICS2D_COLLISION_CCD_H
#define PHYSICS2D_COLLISION_CCD_H
#include "physics2d_detector.h"

#include "physics2d_body.h"
#include "physics2d_grid.h"
#include "physics2d_tree.h"

namespace Physics2D
{
	/// <summary>
	/// Continuous Collision Detection
	///	This class is implemented by bisection and re-sampling. Both them are costly.
	/// </summary>
	class PHYSICS2D_API CCD
	{
	public:
		struct PHYSICS2D_API AABBShot
		{
			AABBShot(const AABB& box, const Body::PhysicsAttribute& attr, const real& t) : aabb(box), attribute(attr),
				time(t)
			{
			}

			AABB aabb;
			Body::PhysicsAttribute attribute;
			real time = 0;
		};

		struct PHYSICS2D_API IndexSection
		{
			int forward = -1;
			int backward = -1;
		};

		struct PHYSICS2D_API CCDPair
		{
			CCDPair() = default;

			CCDPair(const real& time, Body* target) : toi(time), body(target)
			{
			}

			real toi = 0.0;
			Body* body = nullptr;
		};

		using BroadphaseTrajectory = Container::Vector<AABBShot>;
		static std::tuple<BroadphaseTrajectory, AABB> buildTrajectoryAABB(Body* body, const real& dt);
		static std::tuple<Container::Vector<AABBShot>, AABB> buildTrajectoryAABB(
			Body* body, const Vector2& target, const real& dt);
		static std::optional<IndexSection> findBroadphaseRoot(Body* staticBody,
		                                                      const BroadphaseTrajectory& staticTrajectory,
		                                                      Body* dynamicBody,
		                                                      const BroadphaseTrajectory& dynamicTrajectory,
		                                                      const real& dt);
		static std::optional<real> findNarrowphaseRoot(Body* staticBody, const BroadphaseTrajectory& staticTrajectory,
		                                               Body* dynamicBody, const BroadphaseTrajectory& dynamicTrajectory,
		                                               const IndexSection& index, const real& dt);

		static std::optional<Container::Vector<CCDPair>> query(Tree& tree, Body* body, const real& dt);
		static std::optional<Container::Vector<CCDPair>> query(UniformGrid& grid, Body* body, const real& dt);
		static std::optional<real> earliestTOI(const Container::Vector<CCDPair>& pairs,
		                                       const real& epsilon = Constant::GeometryEpsilon);
	};
}
#endif
