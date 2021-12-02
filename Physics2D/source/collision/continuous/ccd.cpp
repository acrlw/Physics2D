#include "../../../include/collision/continuous/ccd.h"

namespace Physics2D
{
	std::tuple<CCD::BroadphaseTrajectory, AABB> CCD::buildTrajectoryAABB(Body* body, const Vector2& target, const real& dt)
	{
		assert(body != nullptr);
		std::vector<AABBShot> trajectory;
		AABB result;
		AABB startBox = AABB::fromBody(body);
		Body::PhysicsAttribute start = body->physicsAttribute();
		body->stepPosition(dt);
		
		Body::PhysicsAttribute end = body->physicsAttribute();
		AABB endBox = AABB::fromBody(body);

		if(startBox == endBox && start.velocity.lengthSquare() < Constant::MaxVelocity && std::fabs(start.angularVelocity) < Constant::MaxAngularVelocity)
		{
			trajectory.emplace_back(AABBShot( startBox, body->physicsAttribute(), 0));
			trajectory.emplace_back(AABBShot( endBox, body->physicsAttribute(), dt ));
			return std::make_tuple(trajectory, result);
		}
		

		result.unite(startBox).unite(endBox);
		
		body->setPhysicsAttribute(start);
		
		real slice = 40;
		real step = dt / slice;
		for(real i = dt / slice;i <= dt;)
		{
			body->stepPosition(step);
			AABB aabb = AABB::fromBody(body);
			trajectory.emplace_back(AABBShot{aabb, body->physicsAttribute(), i});
			result.unite(aabb);
			i += step;
			if ((aabb.position - target).lengthSquare() >= (target - start.position).lengthSquare())
				break;
		}
		body->setPhysicsAttribute(start);
		return std::make_tuple(trajectory, result);
	}
	std::tuple<CCD::BroadphaseTrajectory, AABB> CCD::buildTrajectoryAABB(Body* body, const real& dt)
	{
		assert(body != nullptr);
		std::vector<AABBShot> trajectory;
		AABB result;
		AABB startBox = AABB::fromBody(body);
		Body::PhysicsAttribute start = body->physicsAttribute();
		body->stepPosition(dt);

		Body::PhysicsAttribute end = body->physicsAttribute();
		AABB endBox = AABB::fromBody(body);

		if (startBox == endBox && start.velocity.lengthSquare() < Constant::MaxVelocity && std::fabs(start.angularVelocity) < Constant::MaxAngularVelocity)
		{
			trajectory.emplace_back(AABBShot(startBox, body->physicsAttribute(), 0));
			trajectory.emplace_back(AABBShot(endBox, body->physicsAttribute(), dt));
			return std::make_tuple(trajectory, result);
		}


		result.unite(startBox).unite(endBox);

		body->setPhysicsAttribute(start);

		real slice = 40;
		real step = dt / slice;
		for (real i = dt / slice; i <= dt;)
		{
			body->stepPosition(step);
			AABB aabb = AABB::fromBody(body);
			trajectory.emplace_back(AABBShot{ aabb, body->physicsAttribute(), i });
			result.unite(aabb);
			i += step;
		}
		body->setPhysicsAttribute(start);
		return std::make_tuple(trajectory, result);
	}
	std::optional<CCD::IndexSection> CCD::findBroadphaseRoot(Body* body1, const BroadphaseTrajectory& trajectory1, Body* body2, const BroadphaseTrajectory& trajectory2, const real& dt)
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
			IndexSection result;

			for (auto i = 0; i < length - 1; i++)
			{
				AABB temp1 = AABB::unite(trajectory1[i].aabb, trajectory1[i + 1].aabb);
				AABB temp2 = AABB::unite(trajectory2[i].aabb, trajectory2[i + 1].aabb);
				if (temp1.collide(temp2))
				{
					result.forward = i;
					break;
				}
			}

			return result.forward == -1 ? std::nullopt :
				std::optional(result);
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
			
			IndexSection result;
			size_t length = trajDynamic->size();
			bool forwardFound = false;
			bool backwardFound = false;
			for (size_t i = 0, j = length - 1; i < length - 1; i++, j--)
			{
				AABB tempForward = AABB::unite(trajDynamic->at(i).aabb, trajDynamic->at(i + 1).aabb);
				AABB tempBackward = AABB::unite(trajDynamic->at(j).aabb, trajDynamic->at(j - 1).aabb);
				if (tempForward.collide(traj1) && !forwardFound)
				{
					result.forward = i;
					forwardFound = true;
				}
				if (tempBackward.collide(traj1) && !backwardFound)
				{
					result.backward = j;
					backwardFound = true;
				}
				if (forwardFound && backwardFound)
					break;
			}
			return result.forward == -1 ? std::nullopt :
				std::optional(result);
		}		
		return std::nullopt;
		
	}
	std::optional<real> CCD::findNarrowphaseRoot(Body* body1, const BroadphaseTrajectory& trajectory1, Body* body2, const BroadphaseTrajectory& trajectory2, const IndexSection& index, const real& dt)
	{
		assert(body1 != nullptr && body2 != nullptr);
		bool isBody1CCD = trajectory1.size() > 2;
		bool isBody2CCD = trajectory2.size() > 2;
		if (isBody1CCD && isBody2CCD)
		{
			return std::nullopt;

		}
		if (!isBody1CCD || !isBody2CCD)
		{
			Body* dynamicBody = nullptr;
			Body::PhysicsAttribute origin1 = body1->physicsAttribute();
			Body::PhysicsAttribute origin2 = body2->physicsAttribute();

			real startTimestep = 0;
			real endTimestep = 0;
			if (isBody1CCD)
			{
				dynamicBody = body1;
				body1->setPhysicsAttribute(trajectory1[index.forward].attribute);
				startTimestep = trajectory1[index.forward].time;
				endTimestep = trajectory1[index.backward].time;
			}
			else if (isBody2CCD)
			{
				dynamicBody = body2;
				body2->setPhysicsAttribute(trajectory2[index.forward].attribute);
				startTimestep = trajectory2[index.forward].time;
				endTimestep = trajectory2[index.backward].time;
			}
			//slice maybe 25~70. It depends on how thin the sticks you set
			const real slice = 30.0;
			real step = (endTimestep - startTimestep) / slice;
			real forwardSteps = 0;
			Body::PhysicsAttribute lastAttribute;
			//forwarding
			bool isFound = false;
			while (startTimestep + forwardSteps <= endTimestep)
			{
				lastAttribute = dynamicBody->physicsAttribute();
				dynamicBody->stepPosition(step);
				forwardSteps += step;
				if (const bool result = Detector::collide(body1, body2); result)
				{
					forwardSteps -= step;
					dynamicBody->setPhysicsAttribute(lastAttribute);
					isFound = true;
					break;
				}
			}

			if (!isFound)
			{
				body1->setPhysicsAttribute(origin1);
				body2->setPhysicsAttribute(origin2);
				return std::nullopt;
			}

			//retracing
			step /= 2.0f;
			const real epsilon = 0.01f;
			while (startTimestep + forwardSteps <= endTimestep)
			{
				lastAttribute = dynamicBody->physicsAttribute();
				dynamicBody->stepPosition(step);
				forwardSteps += step;
				if (const auto result = Detector::detect(body1, body2); result.isColliding)
				{
					if (std::fabs(result.penetration) < epsilon)
					{
						body1->setPhysicsAttribute(origin1);
						body2->setPhysicsAttribute(origin2);
						return std::optional(startTimestep + forwardSteps);
					}

					forwardSteps -= step;
					dynamicBody->setPhysicsAttribute(lastAttribute);
					step /= 2.0;
				}
			}
		}

		return std::nullopt;
	}
	std::optional<std::vector<CCD::CCDPair>> CCD::query(DBVH::Node* root, Body* body, const real& dt)
	{
		std::vector<CCDPair> queryList;
		assert(root->isRoot() && body != nullptr);
		auto [trajectoryCCD, aabbCCD] = buildTrajectoryAABB(body, dt);
		std::vector<DBVH::Node*> potential;
		DBVH::queryNodes(root, aabbCCD, potential, body);
		for(DBVH::Node * element: potential)
		{
			auto [trajectoryElement, aabbElement] = buildTrajectoryAABB(element->body, dt);
			auto [newCCDTrajectory, newAABB] = buildTrajectoryAABB(body, element->body->position(), dt);
			auto result = findBroadphaseRoot(body, newCCDTrajectory, element->body, trajectoryElement, dt);
			if(result.has_value())
			{
				auto toi = findNarrowphaseRoot(body, newCCDTrajectory, element->body, trajectoryElement, result.value(), dt);
				if (toi.has_value())
					queryList.emplace_back(CCDPair(toi.value(), element->body));
			}
		}
		return !queryList.empty() ? std::optional(queryList)
			: std::nullopt;
	}
}
