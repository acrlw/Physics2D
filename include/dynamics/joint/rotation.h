#ifndef PHYSICS2D_DYNAMICS_JOINT_ANGLE_H
#define PHYSICS2D_DYNAMICS_JOINT_ANGLE_H
#include "joint.h"
namespace Physics2D
{
	struct RotationJointPrimitive
	{
		Body* bodyA;
		Body* bodyB;
		real referenceRotation = 0;
		real effectiveMass = 0;
		real bias = 0;
		real lastImpulse = 0;
		real accumulatedImpulse = 0;
	};
	class RotationJoint: public Joint
	{
	public:
		RotationJoint()
		{
			m_type = JointType::Rotation;
		}
		RotationJoint(const RotationJointPrimitive& prim) : m_primitive(prim)
		{
			m_type = JointType::Rotation;
		}
		void set(const RotationJointPrimitive& prim)
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
			real c = m_primitive.bodyA->rotation() - m_primitive.bodyB->rotation() - m_primitive.referenceRotation;
			m_primitive.bias = -m_factor * inv_dt * c;
			

		}
		Vector2 solveVelocity(const real& dt) override
		{
			real dw = m_primitive.bodyA->angularVelocity() - m_primitive.bodyB->angularVelocity();
			real impulse = m_primitive.effectiveMass * (-dw + m_primitive.bias);
			//impulse += (impulse - m_primitive.lastImpulse) * m_factor;

			m_primitive.bodyA->angularVelocity() += m_primitive.bodyA->inverseInertia() * impulse;
			m_primitive.bodyB->angularVelocity() -= m_primitive.bodyB->inverseInertia() * impulse;

			m_primitive.lastImpulse = impulse;
			m_primitive.accumulatedImpulse += impulse;
			return Vector2();
		}
		void solvePosition(const real& dt) override
		{

		}
		RotationJointPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		RotationJointPrimitive m_primitive;
		real m_factor = 0.2;
	};
}
#endif