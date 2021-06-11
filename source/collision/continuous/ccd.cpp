#include "include/collision/continuous/ccd.h"

namespace Physics2D
{
	std::tuple<CCD::BroadphaseTrajectory, AABB> CCD::buildTrajectoryAABB(Body* body, const real& dt)
	{
		assert(body != nullptr);
		//start-end
		std::vector<AABBShot> trajectory;
		AABB result;
		AABB startBox = AABB::fromBody(body);
		Body::PhysicsAttribute start = body->physicsAttribute();
		body->stepPosition(dt);
		
		Body::PhysicsAttribute end = body->physicsAttribute();
		AABB endBox = AABB::fromBody(body);

		if(startBox == endBox && start.velocity.lengthSquare() < Constant::MaxVelocity && abs(start.angularVelocity) < Constant::MaxAngularVelocity)
		{
			trajectory.emplace_back(AABBShot( startBox, body->physicsAttribute(), 0));
			trajectory.emplace_back(AABBShot( endBox, body->physicsAttribute(), dt ));
			return std::make_tuple(trajectory, result);
		}
		
		
		result.unite(startBox).unite(endBox);
		
		body->setPhysicsAttribute(start);
		
		//slice until result does not increase
		real slice = 20.0;
		real step = dt / slice;
		//body->setPhysicsInfo(start);
		for(real i = dt / slice;i <= dt;)
		{
			body->stepPosition(step);
			AABB aabb = AABB::fromBody(body);
			trajectory.emplace_back(AABBShot{aabb, body->physicsAttribute(), i});
			result.unite(aabb);
			i += step;
		}
		body->setPhysicsAttribute(start);
		return std::make_tuple(trajectory, result);
	}
	std::optional<real> CCD::findBroadphaseRoot(Body* body1, Body* body2,const BroadphaseTrajectory& trajectory1, const BroadphaseTrajectory& trajectory2, const real& dt)
	{
		AABBShot tNot1, tNot2;
		AABBShot tOverlap1, tOverlap2;
		bool isBody1CCD = trajectory1.size() > 2;
		bool isBody2CCD = trajectory2.size() > 2;
		if (isBody1CCD && isBody2CCD)
		{
			AABB traj1 = AABB::unite(trajectory1[0].aabb, trajectory1[trajectory1.size() - 1].aabb);
			AABB traj2 = AABB::unite(trajectory2[0].aabb, trajectory2[trajectory2.size() - 1].aabb);
			if (!traj1.collide(traj2))
				return std::nullopt;

			for (auto i = 0; i < trajectory1.size(); i++)
			{

			}
		}
		else if (!isBody1CCD || !isBody2CCD)
		{
			const BroadphaseTrajectory* trajStatic = nullptr;
			const BroadphaseTrajectory* trajDynamic = nullptr;
			if (isBody1CCD)
			{
				trajStatic = &trajectory2;
				trajDynamic = &trajectory1;
			}
			else if (isBody2CCD)
			{
				trajStatic = &trajectory1;
				trajDynamic = &trajectory2;
			}

			AABB traj1 = AABB::unite(trajStatic->at(0).aabb, trajStatic->at(1).aabb);
			AABB traj2 = AABB::unite(trajDynamic->at(0).aabb, trajDynamic->at(trajDynamic->size() - 1).aabb);
			if (!traj1.collide(traj2))
				return std::nullopt;
			
			int targetIndex = -1;
			for (auto i = 0; i < trajDynamic->size() - 1; i++)
			{
				AABB temp = AABB::unite(trajDynamic->at(i).aabb, trajDynamic->at(i + 1).aabb);
				if (temp.collide(traj1))
				{
					targetIndex = i;
					break;
				}
			}
			if (targetIndex == -1)
				return std::nullopt;

			bool isForwardCollide = trajDynamic->at(targetIndex).aabb.collide(traj1);
			bool isBackwardCollide = trajDynamic->at(targetIndex + 1).aabb.collide(traj1);
			if (!isForwardCollide && isBackwardCollide)
			{

			}
			if (isForwardCollide && isBackwardCollide)
			{

			}

		}
		
		return std::nullopt;
		
	}
	void CCD::findNarrowphaseRoot(Body* body1, Body* body2)
	{
	}
	void CCD::queryNodes(DBVH::Node* node, const AABB& aabb, std::vector<DBVH::Node*>& nodes)
	{
		if (node == nullptr || !aabb.collide(node->pair.aabb))
			return;

		if (node->isBranch())
		{
			queryNodes(node->left, aabb, nodes);
			queryNodes(node->right, aabb, nodes);
			return;
		}

		if (node->isLeaf())
			nodes.emplace_back(node);
	}
	std::tuple<bool, CCD::BroadphaseTrajectory> CCD::queryLeaf(DBVH::Node* node, Body* body, const real& dt)
	{
		assert(node != nullptr && node->isLeaf());
		auto [trajectoryA, aabbA] = buildTrajectoryAABB(body, dt);
		auto [trajectoryB, aabbB] = buildTrajectoryAABB(node->pair.body, dt);

		return std::make_tuple(aabbA.collide(aabbB), trajectoryB);

	}
}
