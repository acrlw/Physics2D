#include "physics2d_contact.h"


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
			if (elem.second.empty())
				continue;

			for (auto&& ccp : elem.second)
			{
				if (!ccp.active)
					continue;

				auto& vcp = ccp.vcp;
				vcp.va = ccp.bodyA->velocity() + Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
				vcp.vb = ccp.bodyB->velocity() + Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
				Vector2 dv = vcp.va - vcp.vb;

				real jvt = vcp.tangent.dot(dv);
				real lambda_t = vcp.effectiveMassTangent * -jvt;

				real maxFriction = ccp.friction * vcp.accumulatedNormalImpulse;
				real newImpulse = Math::clamp(vcp.accumulatedTangentImpulse + lambda_t, -maxFriction, maxFriction);
				lambda_t = newImpulse - vcp.accumulatedTangentImpulse;
				vcp.accumulatedTangentImpulse = newImpulse;

				Vector2 impulse_t = lambda_t * vcp.tangent;

				ccp.bodyA->applyImpulse(impulse_t, vcp.ra);
				ccp.bodyB->applyImpulse(-impulse_t, vcp.rb);
			}

			if(m_velocityBlockSolver && elem.second.size() == 2)
			{
				//start block solver

			}
			else
			{
				for (auto&& ccp : elem.second)
				{
					if (!ccp.active)
						continue;

					auto& vcp = ccp.vcp;

					Vector2 wa = Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
					Vector2 wb = Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
					vcp.va = ccp.bodyA->velocity() + wa;
					vcp.vb = ccp.bodyB->velocity() + wb;

					Vector2 dv = vcp.va - vcp.vb;
					real jv = vcp.normal.dot(dv - vcp.velocityBias);
					real lambda_n = vcp.effectiveMassNormal * -jv;
					real oldImpulse = vcp.accumulatedNormalImpulse;
					vcp.accumulatedNormalImpulse = Math::max(oldImpulse + lambda_n, 0);
					lambda_n = vcp.accumulatedNormalImpulse - oldImpulse;

					Vector2 impulse_n = lambda_n * vcp.normal;

					ccp.bodyA->applyImpulse(impulse_n, vcp.ra);
					ccp.bodyB->applyImpulse(-impulse_n, vcp.rb);
				}
			}

		}
	}

	void ContactMaintainer::solvePosition(real dt)
	{
		for (auto&& elem : m_contactTable)
		{
			if (elem.second.empty() || !elem.second[0].active)
				continue;

			if(m_positionBlockSolver && elem.second.size() == 2)
			{
				//start block solver

			}
			else
			{

				for (auto&& ccp : elem.second)
				{
					auto&& vcp = ccp.vcp;
					Body* bodyA = ccp.bodyA;
					Body* bodyB = ccp.bodyB;
					Vector2 pa = bodyA->toWorldPoint(vcp.localA);
					Vector2 pb = bodyB->toWorldPoint(vcp.localB);
					Vector2 ra = pa - bodyA->position();
					Vector2 rb = pb - bodyB->position();
					Vector2 c = pb - pa;

					const real bias = Math::max(m_biasFactor * (c.dot(vcp.normal) - m_maxPenetration), 0.0f);

					const real im_a = bodyA->inverseMass();
					const real im_b = bodyB->inverseMass();
					const real ii_a = bodyA->inverseInertia();
					const real ii_b = bodyB->inverseInertia();

					const real rn_a = ra.cross(vcp.normal);
					const real rn_b = rb.cross(vcp.normal);

					const real kNormal = im_a + ii_a * rn_a * rn_a +
						im_b + ii_b * rn_b * rn_b;

					vcp.effectiveMassNormal = realEqual(kNormal, 0.0f) ? 0 : 1.0f / kNormal;

					real lambda = vcp.effectiveMassNormal * bias;

					Vector2 impulse = lambda * vcp.normal;

					bodyA->position() += bodyA->inverseMass() * impulse;
					bodyA->rotation() += bodyA->inverseInertia() * ra.cross(impulse);

					bodyB->position() -= bodyB->inverseMass() * impulse;
					bodyB->rotation() -= bodyB->inverseInertia() * rb.cross(impulse);
				}
			}
		}
	}

	void ContactMaintainer::add(const Collision& collision)
	{
		const Body* bodyA = collision.bodyA;
		const Body* bodyB = collision.bodyB;
		const auto relation = Body::BodyPair::generateBodyPairID(collision.bodyA, collision.bodyB);
		auto& contactList = m_contactTable[relation];

		for (uint8_t i = 0; i < collision.contactList.count; i += 2)
		{
			VertexPair elem;
			elem.pointA = collision.contactList.points[i];
			elem.pointB = collision.contactList.points[i + 1];

			bool existed = false;
			Vector2 localA = bodyA->toLocalPoint(elem.pointA);
			Vector2 localB = bodyB->toLocalPoint(elem.pointB);
			for (auto& contact : contactList)
			{
				const bool isPointA = localA.fuzzyEqual(contact.localA, Constant::TrignometryEpsilon);
				const bool isPointB = localB.fuzzyEqual(contact.localB, Constant::TrignometryEpsilon);
				
				if (isPointA || isPointB)
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
		if(m_velocityBlockSolver && collision.contactList.count == 2)
		{
			//start block solver
			auto& vcp1 = contactList[0].vcp;
			auto& vcp2 = contactList[1].vcp;

			real rn1A = vcp1.ra.cross(collision.normal);
			real rn1B = vcp1.rb.cross(collision.normal);
			real rn2A = vcp2.ra.cross(collision.normal);
			real rn2B = vcp2.rb.cross(collision.normal);


			real k11 = bodyA->inverseMass() + bodyA->inverseInertia() * rn1A * rn1A +
				bodyB->inverseMass() + bodyB->inverseInertia() * rn1B * rn1B;
			real k22 = bodyA->inverseMass() + bodyA->inverseInertia() * rn2A * rn2A +
				bodyB->inverseMass() + bodyB->inverseInertia() * rn2B * rn2B;
			real k12 = bodyA->inverseMass() + bodyA->inverseInertia() * rn1A * rn2A +
				bodyB->inverseMass() + bodyB->inverseInertia() * rn1B * rn2B;

				
			if(k11 * k11 < 1000.0f * (k11 * k22 - k12 * k12))
			{
				Matrix2x2 k(k11, k12, k12, k22);
				contactList[0].k = k;
				contactList[1].k = k;
				k.invert();
				contactList[0].normalMass = k;
				contactList[1].normalMass = k;
			}
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
			const auto& [key, value] = item;
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

	void ContactMaintainer::prepare(ContactConstraintPoint& ccp, const VertexPair& pair, const Collision& collision)
	{
		ccp.bodyA = collision.bodyA;
		ccp.bodyB = collision.bodyB;
		ccp.active = true;

		ccp.friction = Math::sqrt(ccp.bodyA->friction() * ccp.bodyB->friction());

		VelocityConstraintPoint& vcp = ccp.vcp;
		vcp.localA = collision.bodyA->toLocalPoint(pair.pointA);
		vcp.localB = collision.bodyB->toLocalPoint(pair.pointB);

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
		if (m_warmStart)
		{
			ccp.bodyA->applyImpulse(impulse, vcp.ra);
			ccp.bodyB->applyImpulse(-impulse, vcp.rb);
		}
	}
}
