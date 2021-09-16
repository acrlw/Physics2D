#include "include/dynamics/constraint/contact.h"

namespace Physics2D
{
	RelationID generateRelation(Body* bodyA, Body* bodyB)
	{
		return std::stol(std::to_string(bodyA->id()) + std::to_string(bodyB->id()));
	}
	
	void ContactMaintainer::solve(real dt)
	{
		std::vector<int> removedList;
		for (auto iter = m_contactTable.begin(); iter != m_contactTable.end(); ++iter)
		{
			for (auto iterInner = iter->second.begin(); iterInner != iter->second.end(); ++iterInner)
				if (!iterInner->active)
					removedList.push_back(iterInner->contactId);
			for (auto id : removedList)
			{
				for (auto removed = iter->second.begin(); removed != iter->second.end(); ++removed)
				{
					if (removed->contactId == id)
					{
						iter->second.erase(removed);
						break;
					}
				}
			}
			removedList.clear();
		}
		for (auto iter = m_contactTable.begin(); iter != m_contactTable.end(); ++iter)
		{
			if (iter->second.size() == 0 || !iter->second[0].active)
				continue;
			switch (iter->second.size())
			{
			case 1:
			{
				auto& ccp = iter->second[0];

				auto& vcp = ccp.vcp;
				Vector2 dv = vcp.vb - vcp.va;
				real jv = vcp.normal.dot(dv);
				real jvb = jv + vcp.bias;
				real lambda_n = vcp.effectiveMassNormal * jvb;
				//lambda_n = Math::max(lambda_n, 0);

				real oldImpulse = vcp.accumulatedNormalImpulse;
				vcp.accumulatedNormalImpulse = Math::max(oldImpulse + lambda_n, 0);
				lambda_n = vcp.accumulatedNormalImpulse - oldImpulse;


				Vector2 impulse_n = lambda_n * vcp.normal;


				ccp.bodyA->velocity() += ccp.bodyA->inverseMass() * impulse_n;
				ccp.bodyA->angularVelocity() += ccp.bodyA->inverseInertia() * vcp.ra.cross(impulse_n);

				ccp.bodyB->velocity() += ccp.bodyB->inverseMass() * -impulse_n;
				ccp.bodyB->angularVelocity() += ccp.bodyB->inverseInertia() * vcp.rb.cross(-impulse_n);
				

				ccp.active = false;
				break;
			}
			case 2:
			{
				auto* ccp1 = &iter->second[0];
				auto* ccp2 = &iter->second[1];
				break;
			}
			}


		}
	}

	void ContactMaintainer::add(const Collision& collision)
	{
		const Body* bodyA = collision.bodyA;
		const Body* bodyB = collision.bodyB;
		const auto relation = generateRelation(collision.bodyA, collision.bodyB);
		auto& contactList = m_contactTable[relation];
		assert(contactList.size() <= 2);
		for(const auto& elem: collision.contactList)
		{
			Vector2 localA = bodyA->toLocalPoint(elem.pointA);
			Vector2 localB = bodyB->toLocalPoint(elem.pointB);
			for(auto& contact: contactList)
			{
				const bool isPointA = localA.fuzzyEqual(contact.localA, 0.1);
				const bool isPointB = localB.fuzzyEqual(contact.localB, 0.1);
				if(isPointA && isPointB)
				{
					//satisfy the condition, transmit the old accumulated value to new value
					contact.localA = localA;
					contact.localB = localB;
					renewCcp(contact, elem, collision);
					return;
				}
			}
			//no eligible contact, push new contact points
			ContactConstraintPoint ccp;
			ccp.localA = localA;
			ccp.localB = localB;
			ccp.relation = relation;
			renewCcp(ccp, elem, collision);
			contactList.emplace_back(ccp);
		}
	}

	void ContactMaintainer::renewCcp(ContactConstraintPoint& ccp, const PointPair& pair, const Collision& collision)
	{
		ccp.bodyA = collision.bodyA;
		ccp.bodyB = collision.bodyB;
		ccp.active = true;

		ccp.friction = Math::sqrt(ccp.bodyA->friction() * ccp.bodyB->friction());

		VelocityConstraintPoint& vcp = ccp.vcp;
		vcp.ra = pair.pointA - collision.bodyA->position();
		vcp.rb = pair.pointB - collision.bodyB->position();
		Vector2 wa = Vector2::crossProduct(collision.bodyA->angularVelocity(), vcp.ra);
		Vector2 wb = Vector2::crossProduct(collision.bodyB->angularVelocity(), vcp.rb);
		vcp.va = collision.bodyA->velocity() + wa;
		vcp.vb = collision.bodyB->velocity() + wb;

		vcp.normal = collision.normal;
		vcp.tangent = vcp.normal.perpendicular();

		const real im_a = collision.bodyA->inverseMass();
		const real im_b = collision.bodyB->inverseMass();
		const real ii_a = collision.bodyA->inverseInertia();
		const real ii_b = collision.bodyB->inverseInertia();

		const real rn_a = vcp.ra.cross(vcp.normal);
		const real rn_b = vcp.rb.cross(vcp.normal);

		const real rt_a = vcp.ra.cross(vcp.tangent);
		const real rt_b = vcp.rb.cross(vcp.tangent);

		const real kNormal = im_a + ii_a * rn_a * rn_a +
			im_b + ii_b * rn_b * rn_b;

		const real kTangent = im_a + ii_a * rt_a * rt_a +
			im_b + ii_b * rt_b * rt_b;

		vcp.effectiveMassNormal = realEqual(kNormal, 0.0) ? 0 : 1.0 / kNormal;
		vcp.effectiveMassTangent = realEqual(kTangent, 0.0) ? 0 : 1.0 / kTangent;

		vcp.bias = -m_biasFactor * Math::max(0.0, collision.penetration - m_maxPenetration) / 60.0;

		//accumulate inherited impulse
		Vector2 impulse = vcp.accumulatedNormalImpulse * vcp.normal + vcp.accumulatedTangentImpulse * vcp.tangent;


		ccp.bodyA->velocity() += ccp.bodyA->inverseMass() * impulse;
		ccp.bodyA->angularVelocity() += ccp.bodyA->inverseInertia() * vcp.ra.cross(impulse);

		ccp.bodyB->velocity() += ccp.bodyB->inverseMass() * -impulse;
		ccp.bodyB->angularVelocity() += ccp.bodyB->inverseInertia() * vcp.rb.cross(-impulse);
		


	}
}
