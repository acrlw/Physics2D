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
		real biasFactor = 0.3f;
		real bias = 0.0f;
		real minDistance = 0.0f;
		real maxDistance = 0.0f;
		real effectiveMass = 0.0f;
		real accumulatedImpulse = 0.0f;
	};
	struct DistanceConstraintPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vector2 nearestPointA;
		Vector2 nearestPointB;
		Vector2 ra;
		Vector2 rb;
		Vector2 bias;
		Matrix2x2 effectiveMass;
		Vector2 impulse;
		real maxForce = 200.0f;
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
			m_primitive.effectiveMass = 1.0f / (im_a + ii_a * rn_a * rn_a);
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
			m_primitive.bodyA->applyImpulse(impulse, ra);
			//m_primitive.bodyA->velocity() += m_primitive.bodyA->inverseMass() * impulse;
			//m_primitive.bodyA->angularVelocity() += m_primitive.bodyA->inverseInertia() * ra.cross(impulse);
		}
		void solvePosition(const real& dt) override
		{
			
		}

		DistanceJointPrimitive& primitive()
		{
			return m_primitive;
		}
	private:
		real m_factor = 0.4f;
		DistanceJointPrimitive m_primitive;
	};
	class DistanceConstraint : public Joint
	{
		DistanceConstraint()
		{
		}
		DistanceConstraint(const DistanceConstraintPrimitive& primitive) : m_primitive(primitive)
		{
		}
		void set(const DistanceConstraintPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			Body* bodyA = m_primitive.bodyB;
			Body* bodyB = m_primitive.bodyB;

			real im_a = bodyA->inverseMass();
			real ii_a = bodyA->inverseInertia();

			real im_b = bodyB->inverseMass();
			real ii_b = bodyB->inverseInertia();

			m_primitive.ra = m_primitive.nearestPointA - bodyA->position();
			m_primitive.rb = m_primitive.nearestPointB - bodyB->position();
			Vector2& ra = m_primitive.ra;
			Vector2& rb = m_primitive.rb;
			Vector2 error = m_primitive.nearestPointA - m_primitive.nearestPointB;

			Matrix2x2 k;
			k.e11() = im_a + ra.y * ra.y * ii_a + im_b + rb.y * rb.y * ii_b;
			k.e12() = -ra.x * ra.y * ii_a - rb.x * rb.y * ii_b;
			k.e21() = k.e12();
			k.e22() = im_a + ra.x * ra.x * ii_a + im_b + rb.x * rb.x * ii_b;
			m_primitive.bias = error * m_factor;
			m_primitive.effectiveMass = k.invert();

		}
		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			Vector2 va = m_primitive.bodyA->velocity() + Vector2::crossProduct(m_primitive.bodyA->angularVelocity(), m_primitive.ra);
			Vector2 vb = m_primitive.bodyB->velocity() + Vector2::crossProduct(m_primitive.bodyB->angularVelocity(), m_primitive.rb);

			Vector2 jvb = va - vb;
			jvb += m_primitive.bias;
			jvb.negate();
			Vector2 J = m_primitive.effectiveMass.multiply(jvb);
			Vector2 oldImpulse = m_primitive.impulse;
			m_primitive.impulse += J;
			real maxImpulse = dt * m_primitive.maxForce;
			if (m_primitive.impulse.lengthSquare() > maxImpulse * maxImpulse)
			{
				m_primitive.impulse.normalize();
				m_primitive.impulse *= maxImpulse;
			}
			J = m_primitive.impulse - oldImpulse;
			m_primitive.bodyA->applyImpulse(J, m_primitive.ra);
			m_primitive.bodyB->applyImpulse(-J, m_primitive.rb);

		}
		void set(const Vector2& pointA, const Vector2& pointB)
		{
			m_primitive.nearestPointA = pointA;
			m_primitive.nearestPointB = pointB;
		}
		void solvePosition(const real& dt) override
		{


		}

		DistanceConstraintPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		DistanceConstraintPrimitive m_primitive;
		real m_factor = 0.1f;
	};
}
#endif