#include "contact.h"

namespace Physics2D
{



	void ContactMaintainer::clearAll()
	{
		m_contactTable.clear();
	}

	void ContactMaintainer::solveVelocity(real dt)
	{
		for (auto&& elem : m_contactTable)
		{
			if (elem.second.empty() || !elem.second[0].active)
				continue;
			for (auto&& ccp : elem.second)
			{
				auto& vcp = ccp.vcp;

				Vector2 wa = Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
				Vector2 wb = Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
				vcp.va = ccp.bodyA->velocity() + wa;
				vcp.vb = ccp.bodyB->velocity() + wb;

				Vector2 dv = vcp.va - vcp.vb;
				real jv = -1.0f * vcp.normal.dot(dv - vcp.velocityBias);
				real lambda_n = vcp.effectiveMassNormal * jv;
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
				real lambda_t = vcp.effectiveMassTangent * -jvt;

				real maxT = ccp.friction * vcp.accumulatedNormalImpulse;
				oldImpulse = vcp.accumulatedTangentImpulse;
				vcp.accumulatedTangentImpulse = Math::clamp(oldImpulse + lambda_t, -maxT, maxT);

				lambda_t = vcp.accumulatedTangentImpulse - oldImpulse;

				Vector2 impulse_t = lambda_t * vcp.tangent;


				ccp.bodyA->applyImpulse(impulse_t, vcp.ra);
				ccp.bodyB->applyImpulse(-impulse_t, vcp.rb);

			}
		}
	}

	void ContactMaintainer::solvePosition(real dt)
	{
		for (auto&& elem : m_contactTable)
		{
			if (elem.second.empty() || !elem.second[0].active)
				continue;
			for (auto&& ccp : elem.second)
			{
				auto&& vcp = ccp.vcp;
				Body* bodyA = ccp.bodyA;
				Body* bodyB = ccp.bodyB;
				Vector2 pa = vcp.ra + bodyA->position();
				Vector2 pb = vcp.rb + bodyB->position();
				Vector2 c = pb - pa;

				real bias = m_biasFactor * Math::max(c.length() - m_maxPenetration, 0.0f);
				real lambda = vcp.effectiveMassNormal * bias;

				Vector2 impulse = lambda * vcp.normal;

				if (bodyA->type() != Body::BodyType::Static && !bodyA->sleep())
				{
					bodyA->position() += bodyA->inverseMass() * impulse;
					bodyA->rotation() += bodyA->inverseInertia() * vcp.ra.cross(impulse);
				}
				if (bodyB->type() != Body::BodyType::Static && !bodyB->sleep())
				{
					bodyB->position() -= bodyB->inverseMass() * impulse;
					bodyB->rotation() -= bodyB->inverseInertia() * vcp.rb.cross(impulse);
				}
			}
		}
	}

	void ContactMaintainer::add(const Collision& collision)
	{
		const Body* bodyA = collision.bodyA;
		const Body* bodyB = collision.bodyB;
		const auto relation = Body::Relation::generateRelationID(collision.bodyA, collision.bodyB);
		auto& contactList = m_contactTable[relation];
		//assert(contactList.size() <= 2);

		for (const auto& elem : collision.contactList)
		{
			bool existed = false;
			Vector2 localA = bodyA->toLocalPoint(elem.pointA);
			Vector2 localB = bodyB->toLocalPoint(elem.pointB);
			for (auto& contact : contactList)
			{
				const bool isPointA = localA.fuzzyEqual(contact.localA, 0.1f);
				const bool isPointB = localB.fuzzyEqual(contact.localB, 0.1f);
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

	void ContactMaintainer::clearInactivePoints()
	{
		for (auto&& iter : m_contactTable)
		{
			auto& contactList = iter.second;
			std::erase_if(contactList, [](const ContactConstraintPoint& ccp)
			{
				return !ccp.active;
			});
		}
		std::erase_if(m_contactTable, [](const auto& item)
			{
				auto const& [key, value] = item;
				return value.empty();
			});
		
	}

	void ContactMaintainer::deactivateAllPoints()
	{
		for (auto iter = m_contactTable.begin(); iter != m_contactTable.end(); ++iter)
		{
			if (iter->second.empty() || !iter->second[0].active)
				continue;

			for (auto& ccp : iter->second)
				ccp.active = false;
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

		vcp.effectiveMassNormal = realEqual(kNormal, 0.0f) ? 0 : 1.0f / kNormal;
		vcp.effectiveMassTangent = realEqual(kTangent, 0.0f) ? 0 : 1.0f / kTangent;

		//vcp.bias = 0;
		vcp.restitution = Math::min(ccp.bodyA->restitution(), ccp.bodyB->restitution());
		vcp.penetration = collision.penetration;

		Vector2 wa = Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
		Vector2 wb = Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
		vcp.va = ccp.bodyA->velocity() + wa;
		vcp.vb = ccp.bodyB->velocity() + wb;

		vcp.velocityBias = -vcp.restitution * (vcp.va - vcp.vb);

		//accumulate inherited impulse
		Vector2 impulse = vcp.accumulatedNormalImpulse * vcp.normal + vcp.accumulatedTangentImpulse * vcp.tangent;
		//Vector2 impulse;
		ccp.bodyA->applyImpulse(impulse, vcp.ra);
		ccp.bodyB->applyImpulse(-impulse, vcp.rb);
		


	}
}
