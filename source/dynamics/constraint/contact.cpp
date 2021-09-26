#include "include/dynamics/constraint/contact.h"

namespace Physics2D
{
	RelationID generateRelation(Body* bodyA, Body* bodyB)
	{
		return std::stol(std::to_string(bodyA->id()) + std::to_string(bodyB->id()));
	}
	

	void ContactMaintainer::solve(real dt)
	{
		std::vector<int> removedList, clearList;
		for (auto iter = m_contactTable.begin(); iter != m_contactTable.end(); ++iter)
		{
			if (iter->second.size() == 0)
			{
				clearList.push_back(iter->first);
				continue;
			}

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

		for(auto id: clearList)
		{
			for (auto iter = m_contactTable.begin(); iter != m_contactTable.end(); ++iter)
			{
				if(iter->first == id)
				{
					m_contactTable.erase(iter);
					break;
				}
			}
		}

		

		for (auto iter = m_contactTable.begin(); iter != m_contactTable.end(); ++iter)
		{
			if (iter->second.size() == 0 || !iter->second[0].active)
				continue;

			for(auto& ccp: iter->second)
			{
				auto& vcp = ccp.vcp;


				Vector2 wa = Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
				Vector2 wb = Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
				vcp.va = ccp.bodyA->velocity() + wa;
				vcp.vb = ccp.bodyB->velocity() + wb;

				Vector2 dv = vcp.va - vcp.vb;
				real jv = vcp.normal.dot(dv);
				real jvb = - vcp.restitution * jv + vcp.bias;
				real lambda_n = vcp.effectiveMassNormal * jvb;
				real oldImpulse = vcp.accumulatedNormalImpulse;
				vcp.accumulatedNormalImpulse = Math::max(oldImpulse + lambda_n, 0);
				lambda_n = vcp.accumulatedNormalImpulse - oldImpulse;

				Vector2 impulse_n = lambda_n * vcp.normal;

				ccp.bodyA->applyImpulse(impulse_n, vcp.ra);
				ccp.bodyB->applyImpulse(-impulse_n, vcp.rb);

				vcp.va = ccp.bodyA->velocity() + Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
				vcp.vb = ccp.bodyB->velocity() + Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
				dv = vcp.va - vcp.vb;

				real jvt = vcp.tangent.dot(dv);
				real lambda_t = vcp.effectiveMassTangent * - jvt;


				real maxT = ccp.friction * vcp.accumulatedNormalImpulse;
				oldImpulse = vcp.accumulatedTangentImpulse;
				vcp.accumulatedTangentImpulse = Math::clamp(oldImpulse + lambda_t, -maxT, maxT);

				lambda_t = vcp.accumulatedTangentImpulse - oldImpulse;

				Vector2 impulse_t = lambda_t * vcp.tangent;


				ccp.bodyA->applyImpulse(impulse_t, vcp.ra);
				ccp.bodyB->applyImpulse(-impulse_t, vcp.rb);

				ccp.active = false;
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
		for (const auto& elem : collision.contactList)
		{
			bool existed = false;
			Vector2 localA = bodyA->toLocalPoint(elem.pointA);
			Vector2 localB = bodyB->toLocalPoint(elem.pointB);
			for (auto& contact : contactList)
			{
				const bool isPointA = localA.fuzzyEqual(contact.localA, 0.2);
				const bool isPointB = localB.fuzzyEqual(contact.localB, 0.2);
				if (isPointA && isPointB)
				{
					//satisfy the condition, transmit the old accumulated value to new value
					contact.localA = localA;
					contact.localB = localB;
					prepare(contact, elem, collision);
					existed = true;
					break;
				}
			}
			if (existed)
				continue;
			//no eligible contact, push new contact points
			ContactConstraintPoint ccp;
			ccp.localA = localA;
			ccp.localB = localB;
			ccp.relation = relation;
			prepare(ccp, elem, collision);
			contactList.emplace_back(ccp);
		}
	}

	void ContactMaintainer::prepare(ContactConstraintPoint& ccp, const PointPair& pair, const Collision& collision)
	{
		ccp.bodyA = collision.bodyA;
		ccp.bodyB = collision.bodyB;
		ccp.active = true;

		ccp.friction = Math::sqrt(ccp.bodyA->friction() * ccp.bodyB->friction());

		VelocityConstraintPoint& vcp = ccp.vcp;
		vcp.ra = pair.pointA - collision.bodyA->position();
		vcp.rb = pair.pointB - collision.bodyB->position();

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

		//vcp.bias = 0;
		vcp.bias = m_biasFactor * Math::max(0.0, collision.penetration - m_maxPenetration) * 60.0;
		vcp.restitution = Math::min(ccp.bodyA->restitution(), ccp.bodyB->restitution());
		//accumulate inherited impulse
		Vector2 impulse = vcp.accumulatedNormalImpulse * vcp.normal + vcp.accumulatedTangentImpulse * vcp.tangent;
		//Vector2 impulse;
		ccp.bodyA->applyImpulse(impulse, vcp.ra);
		ccp.bodyB->applyImpulse(-impulse, vcp.rb);
		


	}
}
