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
	};
	struct OrientationJointPrimitive
	{
		Body* bodyA;
		Vector2 targetPoint;
		real referenceRotation = 0;
		real bias = 0;
		real effectiveMass = 0;
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
			real inv_dt = 1.0f / dt;
			m_primitive.effectiveMass = 1.0f / (ii_a + ii_b);
			real c = m_primitive.bodyA->rotation() - m_primitive.bodyB->rotation() - m_primitive.referenceRotation;
			m_primitive.bias = -m_factor * inv_dt * c;
		}
		void solveVelocity(const real& dt) override
		{
			real dw = m_primitive.bodyA->angularVelocity() - m_primitive.bodyB->angularVelocity();
			real impulse = m_primitive.effectiveMass * (-dw + m_primitive.bias);

			m_primitive.bodyA->angularVelocity() += m_primitive.bodyA->inverseInertia() * impulse;
			m_primitive.bodyB->angularVelocity() -= m_primitive.bodyB->inverseInertia() * impulse;
			
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
		real m_factor = 0.2f;
	};
	class OrientationJoint : public Joint
	{

	public:
		OrientationJoint()
		{
			m_type = JointType::Orientation;
		}
		OrientationJoint(const OrientationJointPrimitive& prim) : m_primitive(prim)
		{
			m_type = JointType::Orientation;
		}
		void set(const OrientationJointPrimitive& prim)
		{
			m_primitive = prim;
		}
		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr)
				return;

			Body* bodyA = m_primitive.bodyA;
			Vector2 point = m_primitive.targetPoint - bodyA->position();
			real targetRotation = point.theta();

			real ii_a = m_primitive.bodyA->inverseInertia();
			real inv_dt = 1.0f / dt;
			m_primitive.effectiveMass = 1.0f / ii_a;
			real c = targetRotation - m_primitive.bodyA->rotation() - m_primitive.referenceRotation;
			if(fuzzyRealEqual(c, 2.0f * Constant::Pi, 0.1f))
			{
				c = 0;
				bodyA->rotation() = targetRotation;
				return;
			}
			if (fuzzyRealEqual(c, -2.0f * Constant::Pi, 0.1f))
			{
				c = 0;
				bodyA->rotation() = targetRotation;
				return;
			}
			m_primitive.bias = m_factor * inv_dt * c;
		}
		void solveVelocity(const real& dt) override
		{
			real dw = m_primitive.bodyA->angularVelocity();
			real impulse = m_primitive.effectiveMass * (-dw + m_primitive.bias);

			m_primitive.bodyA->angularVelocity() += m_primitive.bodyA->inverseInertia() * impulse;

		}
		void solvePosition(const real& dt) override
		{

		}
		OrientationJointPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		OrientationJointPrimitive m_primitive;
		real m_factor = 1.0;
	};
}
#endif