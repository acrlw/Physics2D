#ifndef PHYSICS2D_DYNAMICS_JOINT_ANGLE_H
#define PHYSICS2D_DYNAMICS_JOINT_ANGLE_H
#include "joint.h"
namespace Physics2D
{
	struct AngleJointPrimitive
	{
		Body* bodyA;
		Body* bodyB;
		real referenceAngle = 0;
		real effectiveMass = 0;
		real bias = 0;
		real lastImpulse = 0;
		real accumulatedImpulse = 0;
	};
	class AngleJoint: public Joint
	{
	public:
		AngleJoint()
		{
			m_type = JointType::Angle;
		}
		AngleJoint(const AngleJointPrimitive& prim) : m_primitive(prim)
		{
			m_type = JointType::Angle;
		}
		void set(const AngleJointPrimitive& prim)
		{
			m_primitive = prim;
		}
		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			real ii_a = m_primitive.bodyA->inverseInertia();
			real ii_b = m_primitive.bodyB->inverseInertia();
			real inv_dt = 1.0 / dt;
			m_primitive.effectiveMass = 1.0 / (ii_a + ii_b);
			real c = m_primitive.bodyA->angle() - m_primitive.bodyB->angle() - m_primitive.referenceAngle;
			m_primitive.bias = -m_factor * inv_dt * c;
			

		}
		void solveVelocity(const real& dt) override
		{
			real dw = m_primitive.bodyA->angularVelocity() - m_primitive.bodyB->angularVelocity();
			real impulse = m_primitive.effectiveMass * (-dw + m_primitive.bias);
			//impulse += (impulse - m_primitive.lastImpulse) * m_factor;

			m_primitive.bodyA->angularVelocity() += m_primitive.bodyA->inverseInertia() * impulse;
			m_primitive.bodyB->angularVelocity() -= m_primitive.bodyB->inverseInertia() * impulse;

			m_primitive.lastImpulse = impulse;
			m_primitive.accumulatedImpulse += impulse;
		}
		void solvePosition(const real& dt) override
		{

		}
		AngleJointPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		AngleJointPrimitive m_primitive;
		real m_factor = 0.2;
	};
}
#endif