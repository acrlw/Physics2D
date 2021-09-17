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


				Vector2 wa = Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
				Vector2 wb = Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
				vcp.va = ccp.bodyA->velocity() + wa;
				vcp.vb = ccp.bodyB->velocity() + wb;

				Vector2 dv = vcp.vb - vcp.va;
				real jv = vcp.normal.dot(dv);
				real jvb = jv + vcp.bias;
				real lambda_n = vcp.effectiveMassNormal * jvb;

				real oldImpulse = vcp.accumulatedNormalImpulse;
				vcp.accumulatedNormalImpulse = Math::max(oldImpulse + lambda_n, 0);
				lambda_n = vcp.accumulatedNormalImpulse - oldImpulse;

				Vector2 impulse_n = lambda_n * vcp.normal;

				ccp.bodyA->applyImpulse(impulse_n, vcp.ra);
				ccp.bodyB->applyImpulse(-impulse_n, vcp.rb);

				vcp.va = ccp.bodyA->velocity() + Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
				vcp.vb = ccp.bodyB->velocity() + Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
				dv = vcp.vb - vcp.va;

				real jvt = vcp.tangent.dot(dv);
				real lambda_t = vcp.effectiveMassTangent * jvt;

				
				real maxT = ccp.friction * vcp.accumulatedNormalImpulse;
				oldImpulse = vcp.accumulatedTangentImpulse;
				vcp.accumulatedTangentImpulse = Math::clamp(oldImpulse + lambda_t, -maxT, maxT);

				lambda_t = vcp.accumulatedTangentImpulse - oldImpulse;

				Vector2 impulse_t = lambda_t * vcp.tangent;


				ccp.bodyA->applyImpulse(impulse_t, vcp.ra);
				ccp.bodyB->applyImpulse(-impulse_t, vcp.rb);

				ccp.active = false;
				break;
			}
			case 2:
			{
				auto& ccp1 = iter->second[0];
				auto& ccp2 = iter->second[1];
				auto& vcp1 = ccp1.vcp;
				auto& vcp2 = ccp2.vcp;

				
				vcp1.va = ccp1.bodyA->velocity() + Vector2::crossProduct(ccp1.bodyA->angularVelocity(), vcp1.ra);
				vcp1.vb = ccp1.bodyB->velocity() + Vector2::crossProduct(ccp1.bodyB->angularVelocity(), vcp1.rb);
				
				vcp2.va = ccp2.bodyA->velocity() + Vector2::crossProduct(ccp2.bodyA->angularVelocity(), vcp2.ra);
				vcp2.vb = ccp2.bodyB->velocity() + Vector2::crossProduct(ccp2.bodyB->angularVelocity(), vcp2.rb);

				Vector2 dv1 = vcp1.vb - vcp1.va;
				Vector2 dv2 = vcp2.vb - vcp2.va;

				real jv1 = vcp1.normal.dot(dv1);
				real jv2 = vcp2.normal.dot(dv2);
				real jvb1 = jv1 + vcp1.bias;
				real jvb2 = jv2 + vcp2.bias;


				real lambda_n1 = vcp1.effectiveMassNormal * jvb1;
				real lambda_n2 = vcp2.effectiveMassNormal * jvb2;

				real oldImpulse1 = vcp1.accumulatedNormalImpulse;
				vcp1.accumulatedNormalImpulse = Math::max(oldImpulse1 + lambda_n1, 0);
				lambda_n1 = vcp1.accumulatedNormalImpulse - oldImpulse1;

				real oldImpulse2 = vcp2.accumulatedNormalImpulse;
				vcp2.accumulatedNormalImpulse = Math::max(oldImpulse2 + lambda_n2, 0);
				lambda_n2 = vcp2.accumulatedNormalImpulse - oldImpulse2;


				Vector2 impulse_n = (lambda_n1 * vcp1.normal + lambda_n2 * vcp2.normal);

				ccp1.bodyA->applyImpulse(impulse_n, vcp1.ra + vcp2.ra);
				ccp1.bodyB->applyImpulse(-impulse_n, vcp1.rb + vcp2.rb);


				vcp1.va = ccp1.bodyA->velocity() + Vector2::crossProduct(ccp1.bodyA->angularVelocity(), vcp1.ra);
				vcp1.vb = ccp1.bodyB->velocity() + Vector2::crossProduct(ccp1.bodyB->angularVelocity(), vcp1.rb);

				vcp2.va = ccp2.bodyA->velocity() + Vector2::crossProduct(ccp2.bodyA->angularVelocity(), vcp2.ra);
				vcp2.vb = ccp2.bodyB->velocity() + Vector2::crossProduct(ccp2.bodyB->angularVelocity(), vcp2.rb);

				dv1 = vcp1.vb - vcp1.va;
				dv2 = vcp2.vb - vcp2.va;

				real jvt1 = vcp1.tangent.dot(dv1);
				real lambda_t1 = vcp1.effectiveMassTangent * jvt1;

				real jvt2 = vcp2.tangent.dot(dv2);
				real lambda_t2 = vcp2.effectiveMassTangent * jvt2;

				
				real maxT1 = ccp1.friction * vcp1.accumulatedNormalImpulse;
				oldImpulse1 = vcp1.accumulatedTangentImpulse;
				vcp1.accumulatedTangentImpulse = Math::clamp(oldImpulse1 + lambda_t1, -maxT1, maxT1);
				lambda_t1 = vcp1.accumulatedTangentImpulse - oldImpulse1;

				real maxT2 = ccp2.friction * vcp2.accumulatedNormalImpulse;
				oldImpulse2 = vcp2.accumulatedTangentImpulse;
				vcp2.accumulatedTangentImpulse = Math::clamp(oldImpulse2 + lambda_t2, -maxT2, maxT2);
				lambda_t2 = vcp2.accumulatedTangentImpulse - oldImpulse2;

				Vector2 impulse_t = lambda_t1 * vcp1.tangent + lambda_t2 * vcp2.tangent;

				ccp1.bodyA->applyImpulse(impulse_t, vcp1.ra + vcp2.ra);
				ccp1.bodyB->applyImpulse(-impulse_t, vcp1.rb + vcp2.rb);

				ccp1.active = false;
				ccp2.active = false;
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
			bool existed = false;
			bool two = false;
			Vector2 localA = bodyA->toLocalPoint(elem.pointA);
			Vector2 localB = bodyB->toLocalPoint(elem.pointB);
			if (contactList.size() == 2)
				two = true;
			for(auto& contact: contactList)
			{
				const bool isPointA = localA.fuzzyEqual(contact.localA, 0.2);
				const bool isPointB = localB.fuzzyEqual(contact.localB, 0.2);
				if(isPointA || isPointB)
				{
					//satisfy the condition, transmit the old accumulated value to new value
					contact.localA = localA;
					contact.localB = localB;
					prepare(contact, elem, collision);
					existed = true;
					break;
				}
			}
			if(existed)
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

		vcp.bias = m_biasFactor * Math::max(0.0, collision.penetration - m_maxPenetration) * 60.0;
		//accumulate inherited impulse
		Vector2 impulse = vcp.accumulatedNormalImpulse * vcp.normal + vcp.accumulatedTangentImpulse * vcp.tangent;

		ccp.bodyA->applyImpulse(impulse, vcp.ra);
		ccp.bodyB->applyImpulse(-impulse, vcp.rb);
		


	}
}
