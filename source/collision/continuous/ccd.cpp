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
	std::optional<size_t> CCD::findBroadphaseRoot(Body* body1, const BroadphaseTrajectory& trajectory1, Body* body2, const BroadphaseTrajectory& trajectory2, const real& dt)
	{
		assert(body1 != nullptr && body2 != nullptr);
		bool isBody1CCD = trajectory1.size() > 2;
		bool isBody2CCD = trajectory2.size() > 2;
		if (isBody1CCD && isBody2CCD)
		{
			AABB traj1 = AABB::unite(trajectory1[0].aabb, trajectory1[trajectory1.size() - 1].aabb);
			AABB traj2 = AABB::unite(trajectory2[0].aabb, trajectory2[trajectory2.size() - 1].aabb);
			if (!traj1.collide(traj2))
				return std::nullopt;
			size_t length = trajectory1.size() > trajectory2.size() ? trajectory2.size() : trajectory1.size();
			size_t targetIndex = -1;
			for (auto i = 0; i < length; i++)
			{
				AABB temp1 = AABB::unite(trajectory1[i].aabb, trajectory1[i + 1].aabb);
				AABB temp2 = AABB::unite(trajectory2[i].aabb, trajectory2[i + 1].aabb);
				if (temp1.collide(temp2))
				{
					targetIndex = i;
					break;
				}
			}

			return targetIndex == -1 ? std::nullopt : 
				std::optional(targetIndex);
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
			return targetIndex == -1 ? std::nullopt :
				std::optional(targetIndex);
		}		
		return std::nullopt;
		
	}
	std::optional<real> CCD::findNarrowphaseRoot(Body* body1, const BroadphaseTrajectory& trajectory1, Body* body2, const BroadphaseTrajectory& trajectory2, const size_t& index, const real& dt)
	{
		assert(body1 != nullptr && body2 != nullptr);
		bool isBody1CCD = trajectory1.size() > 2;
		bool isBody2CCD = trajectory2.size() > 2;
		if (isBody1CCD && isBody2CCD)
		{
			const size_t startIterTime = trajectory1[index].time;
			const Body::PhysicsAttribute origin1 = body1->physicsAttribute();
			const Body::PhysicsAttribute origin2 = body2->physicsAttribute();
			body1->setPhysicsAttribute(trajectory1[index].attribute);
			body2->setPhysicsAttribute(trajectory2[index].attribute);

			const real slice = 40.0;
			real step = dt / slice;
			real lastTimestep = startIterTime;
			real index = 0;
			while (true)
			{
				body1->stepPosition(step);
				body2->stepPosition(step);
				auto result = Detector::detect(body1, body2);
				if (!result.isColliding)
					continue;

			}
		}
		else if (!isBody1CCD || !isBody2CCD)
		{
			Body* staticBody = nullptr;
			Body* dynamicBody = nullptr;
			Body::PhysicsAttribute origin1 = body1->physicsAttribute();
			Body::PhysicsAttribute origin2 = body2->physicsAttribute();
			Body::PhysicsAttribute startDynamicAttribute;
			real startTimestep = 0;
			real endTimestep = 0;
			if (isBody1CCD)
			{
				dynamicBody = body1;
				body1->setPhysicsAttribute(trajectory1[index].attribute);
				startDynamicAttribute = trajectory1[index].attribute;

				startTimestep = trajectory1[index].time;
				endTimestep = trajectory1[index + 1].time;
			}
			else if (isBody2CCD)
			{
				dynamicBody = body2;
				body2->setPhysicsAttribute(trajectory2[index].attribute);
				startDynamicAttribute = trajectory2[index].attribute;

				startTimestep = trajectory2[index].time;
				endTimestep = trajectory2[index + 1].time;
			}

			const real slice = 100.0;
			real step = dt / slice;
			real steps = 0;
			real epsilon = 0.01;
			Body::PhysicsAttribute lastAttribute;
			
			while (true)
			{
				lastAttribute = dynamicBody->physicsAttribute();
				dynamicBody->stepPosition(step);
				steps += step;
				auto result = Detector::detect(body1, body2);
				if (result.isColliding)
				{
					if (std::abs(result.penetration) < epsilon)
					{
						body1->setPhysicsAttribute(origin1);
						body2->setPhysicsAttribute(origin2);
						return std::optional(startTimestep + steps);
					}

					steps -= step;
					dynamicBody->setPhysicsAttribute(lastAttribute);
					step /= 2.0;
					continue;
				}
				if (startTimestep + steps > endTimestep)
				{
					steps = 0;
					step /= 2.0;
					dynamicBody->setPhysicsAttribute(startDynamicAttribute);
					continue;
				}
			}

		}

		return std::nullopt;
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
