#ifndef PHYSICS2D_DYNAMICS_JOINT_DISTANCE_H
#define PHYSICS2D_DYNAMICS_JOINT_DISTANCE_H
#include "joint.h"
namespace Physics2D
{
	struct DistanceJointPrimitive
	{
		Body* bodyA = nullptr;
		Vector2 localPointA;
		Vector2 targetPoint;
		Vector2 normal;
		real biasFactor = 0.3;
		real bias = 0;
		real minDistance = 0;
		real maxDistance = 0;
		real effectiveMass = 0;
		real accumulatedImpulse = 0;
	};
	class DistanceJoint : public Joint
	{
	public:
		DistanceJoint()
		{
			m_type = JointType::Distance;
		}
		DistanceJoint(const DistanceJointPrimitive& primitive) : m_primitive(primitive)
		{
			m_type = JointType::Distance;
		}
		void set(const DistanceJointPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{
			assert(m_primitive.minDistance <= m_primitive.maxDistance);
			Body* bodyA = m_primitive.bodyA;
			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();
			Vector2 pb = m_primitive.targetPoint;
			real im_a = m_primitive.bodyA->inverseMass();
			real ii_a = m_primitive.bodyA->inverseInertia();
			Vector2 error = pb - pa;
			real length = error.length();
			real c = 0;

			m_primitive.normal = error.normal();
			if (length < m_primitive.minDistance)
			{
				c = m_primitive.minDistance - length;
				m_primitive.normal.negate();
			}
			else if (length > m_primitive.maxDistance)
			{
				c = length - m_primitive.maxDistance;
			}
			else
			{
				m_primitive.accumulatedImpulse = 0;
				m_primitive.normal.clear();
				m_primitive.bias = 0;
				return;
			}
			if (m_primitive.bodyA->velocity().dot(m_primitive.normal) > 0)
			{
				m_primitive.accumulatedImpulse = 0;
				m_primitive.normal.clear();
				m_primitive.bias = 0;
				return;
			}
			real rn_a = m_primitive.normal.dot(ra);
			m_primitive.effectiveMass = 1.0 / (im_a + ii_a * rn_a * rn_a);
 			m_primitive.bias = m_primitive.biasFactor * c / dt;

			//Vector2 impulse = m_primitive.accumulatedImpulse * m_primitive.normal;
			//m_primitive.bodyA->applyImpulse(impulse, ra);
			
		}
		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bias == 0)
				return ;
			Vector2 ra = m_primitive.bodyA->toWorldPoint(m_primitive.localPointA) - m_primitive.bodyA->position();
			Vector2 va = m_primitive.bodyA->velocity() + Vector2::crossProduct(m_primitive.bodyA->angularVelocity(), ra);

			Vector2 dv = va;
			real jv = m_primitive.normal.dot(dv);
			real jvb = -jv + m_primitive.bias;
			real lambda_n = m_primitive.effectiveMass * jvb;

			real oldImpulse = m_primitive.accumulatedImpulse;
			m_primitive.accumulatedImpulse = Math::max(oldImpulse + lambda_n, 0);
			lambda_n = m_primitive.accumulatedImpulse - oldImpulse;

			Vector2 impulse = lambda_n * m_primitive.normal;
			m_primitive.bodyA->velocity() += m_primitive.bodyA->inverseMass() * impulse;
			m_primitive.bodyA->angularVelocity() += m_primitive.bodyA->inverseInertia() * ra.cross(impulse);
		}
		void solvePosition(const real& dt) override
		{
			
		}

		DistanceJointPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		DistanceJointPrimitive m_primitive;
		real m_factor = 0.4;
	};
}
#endif