#include "ccd.h"

namespace Physics2D
{
	std::tuple<CCD::BroadphaseTrajectory, AABB> CCD::buildTrajectoryAABB(Body* body, const Vector2& target, const real& dt)
	{
		assert(body != nullptr);
		Container::Vector<AABBShot> trajectory;
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
		for(real i = step;i <= dt; i += step)
		{
			body->stepPosition(step);
			AABB aabb = AABB::fromBody(body);
			trajectory.emplace_back(AABBShot{aabb, body->physicsAttribute(), i});
			result.unite(aabb);
		}
		body->setPhysicsAttribute(start);
		return std::make_tuple(trajectory, result);
	}
	std::tuple<CCD::BroadphaseTrajectory, AABB> CCD::buildTrajectoryAABB(Body* body, const real& dt)
	{
		assert(body != nullptr);
		Container::Vector<AABBShot> trajectory;
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
	std::optional<CCD::IndexSection> CCD::findBroadphaseRoot(Body* staticBody, const BroadphaseTrajectory& staticTrajectory, Body* dynamicBody, const BroadphaseTrajectory& dynamicTrajectory, const real& dt)
	{
		assert(staticBody != nullptr && dynamicBody != nullptr);


		AABB traj1 = AABB::unite(staticTrajectory[0].aabb, staticTrajectory.back().aabb);
		AABB traj2 = AABB::unite(dynamicTrajectory[0].aabb, dynamicTrajectory.back().aabb);
		if (!traj1.collide(traj2))
			return std::nullopt;

		IndexSection result;
		size_t length = dynamicTrajectory.size();
		bool forwardFound = false;
		bool backwardFound = false;
		for (size_t i = 0, j = length - 1; i < length - 1; i++, j--)
		{
			AABB tempForward = AABB::unite(dynamicTrajectory[i].aabb, dynamicTrajectory[i + 1].aabb);
			AABB tempBackward = AABB::unite(dynamicTrajectory[j].aabb, dynamicTrajectory[j - 1].aabb);
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
	std::optional<real> CCD::findNarrowphaseRoot(Body* staticBody, const BroadphaseTrajectory& staticTrajectory, Body* dynamicBody, const BroadphaseTrajectory& dynamicTrajectory, const IndexSection& index, const real& dt)
	{
		assert(staticBody != nullptr && dynamicBody != nullptr);

		if (dynamicTrajectory.size() < 2)
			return std::nullopt;
		
		Body::PhysicsAttribute staticOrigin = staticBody->physicsAttribute();
		Body::PhysicsAttribute dynamicOrigin = dynamicBody->physicsAttribute();

		real startTimestep = 0;
		real endTimestep = 0;

		dynamicBody->setPhysicsAttribute(dynamicTrajectory[index.forward].attribute);
		startTimestep = dynamicTrajectory[index.forward].time;
		endTimestep = dynamicTrajectory[index.backward].time;

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
			if (const bool result = Detector::collide(staticBody, dynamicBody); result)
			{
				forwardSteps -= step;
				dynamicBody->setPhysicsAttribute(lastAttribute);
				isFound = true;
				break;
			}
		}

		if (!isFound)
		{
			staticBody->setPhysicsAttribute(staticOrigin);
			dynamicBody->setPhysicsAttribute(dynamicOrigin);
			return std::nullopt;
		}

		//retracing
		step /= 2.0f;
		const real epsilon = 0.01f;
		unsigned int counter = 0;
		while (startTimestep + forwardSteps <= endTimestep)
		{
			lastAttribute = dynamicBody->physicsAttribute();
			dynamicBody->stepPosition(step);
			forwardSteps += step;
			if (const auto result = Detector::detect(staticBody, dynamicBody); result.isColliding)
			{
				if (std::fabs(result.penetration) < epsilon || counter >= Constant::CCDMaxIterations)
				{
					staticBody->setPhysicsAttribute(staticOrigin);
					dynamicBody->setPhysicsAttribute(dynamicOrigin);
					return std::optional(startTimestep + forwardSteps);
				}

				forwardSteps -= step;
				dynamicBody->setPhysicsAttribute(lastAttribute);
				step /= 2.0;
			}
			counter++;
		}


		return std::nullopt;
	}
	std::optional<Container::Vector<CCD::CCDPair>> CCD::query(DBVH::Node* root, Body* body, const real& dt)
	{
		Container::Vector<CCDPair> queryList;
		assert(root->isRoot() && body != nullptr);
		auto [trajectoryCCD, aabbCCD] = buildTrajectoryAABB(body, dt);
		Container::Vector<DBVH::Node*> potential;
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

    std::optional<Container::Vector<CCD::CCDPair>> CCD::query(Tree& tree, Body* body, const real& dt)
    {
        Container::Vector<CCDPair> queryList;
        assert(body != nullptr);
        auto [trajectoryCCD, aabbCCD] = buildTrajectoryAABB(body, dt);
        auto potentials = tree.query(aabbCCD);

        for(auto& elem: potentials)
        {
            //skip detecting itself
            if(elem == body)
                continue;

            auto [trajectoryElement, aabbElement] = buildTrajectoryAABB(elem, dt);
            auto [newCCDTrajectory, newAABB] = buildTrajectoryAABB(body, elem->position(), dt);
            auto result = findBroadphaseRoot(elem, trajectoryElement, body, newCCDTrajectory, dt);
            if(result.has_value())
            {
                auto toi = findNarrowphaseRoot(elem, trajectoryElement, body, newCCDTrajectory, result.value(), dt);
                if (toi.has_value())
                    queryList.emplace_back(CCDPair(toi.value(), elem));
            }
        }
        return !queryList.empty() ? std::optional(queryList)
                                  : std::nullopt;
    }

    std::optional<real> CCD::earliestTOI(const Container::Vector<CCDPair> &pairs, const real &epsilon) {
        if(pairs.empty())
            return std::nullopt;

        real minToi = Constant::Max;
		for (const auto& elem : pairs) 
			if (elem.toi < minToi) 
				minToi = elem.toi;

        return minToi;
    }
}
