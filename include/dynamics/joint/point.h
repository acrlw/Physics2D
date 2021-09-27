#ifndef PHYSICS2D_DYNAMICS_JOINT_POINT_H
#define PHYSICS2D_DYNAMICS_JOINT_POINT_H
#include "joint.h"
namespace Physics2D
{
	struct PointJointPrimitive
	{
		Body* bodyA;
		Vector2 localPointA;
		Vector2 targetPoint;
		Vector2 normal;
		real bias = 0;
		real biasFactor = 0.2;
		real effectiveMass = 0;
	};
	class PointJoint : public Joint
	{
	public:
		PointJoint()
		{
			m_type = JointType::Point;
		}
		PointJoint(const PointJointPrimitive& prim) : m_primitive(prim)
		{
			m_type = JointType::Point;
		}
		void set(const PointJointPrimitive& prim)
		{
			m_primitive = prim;
		}
		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();
			Vector2 pb = m_primitive.targetPoint;
			real im_a = m_primitive.bodyA->inverseMass();
			real ii_a = m_primitive.bodyA->inverseInertia();
			Vector2 error = pb - pa;
			m_primitive.normal = error.normal();
			real c = Math::max(error.length() - 0.01, 0);
			real rn_a = m_primitive.normal.dot(ra);
			m_primitive.effectiveMass = 1.0 / (im_a + ii_a * rn_a * rn_a);
			m_primitive.bias = m_primitive.biasFactor * c / dt;
			
		}
		void solveVelocity(const real& dt) override
		{
			Vector2 ra = m_primitive.bodyA->toWorldPoint(m_primitive.localPointA) - m_primitive.bodyA->position();
			Vector2 va = m_primitive.bodyA->velocity() + Vector2::crossProduct(m_primitive.bodyA->angularVelocity(), ra);

			Vector2 dv = va;
			real jv = m_primitive.normal.dot(dv);
			real jvb = -jv + m_primitive.bias;
			real lambda_n = m_primitive.effectiveMass * jvb;
			Vector2 impulse = lambda_n * m_primitive.normal;
			
			m_primitive.bodyA->velocity() += m_primitive.bodyA->inverseMass() * impulse;
			m_primitive.bodyA->angularVelocity() += m_primitive.bodyA->inverseInertia() * ra.cross(impulse);
			
		}
		void solvePosition(const real& dt) override
		{
			
			
		}
		PointJointPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		PointJointPrimitive m_primitive;
		real m_factor = 0.22;
	};
}
#endif