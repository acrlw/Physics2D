#ifndef PHYSICS2D_DYNAMICS_JOINT_WELD_H
#define PHYSICS2D_DYNAMICS_JOINT_WELD_H
#include "joint.h"
namespace Physics2D
{
	struct WeldJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vector2 localPointA;
		Vector2 localPointB;
		real rotation;		//a.rotation() - b.rotation()

		//soft constraint

		real damping = 0.0f;
		real stiffness = 0.0f;
		real frequency = 8.0f;
		real maxForce = 5000.0f;
		real dampingRatio = 0.2f;
		real gamma = 0.0f;
		Vector2 bias;
		Matrix3x3 effectiveMass;
		Vector2 accumulatedImpulse;
	};
	//weld joint comes from revolute and rotation joint
	class WeldJoint : public Joint
	{
	public:
		WeldJoint()
		{
			m_type = JointType::Weld;
		}

		WeldJoint(const WeldJointPrimitive& primitive)
		{
			m_type = JointType::Weld;
			m_primitive = primitive;
		}
		void set(const WeldJointPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			real m_a = bodyA->mass();
			real im_a = bodyA->inverseMass();
			real ii_a = bodyA->inverseInertia();

			real m_b = bodyB->mass();
			real im_b = bodyB->inverseMass();
			real ii_b = bodyB->inverseInertia();


			if (m_primitive.frequency > 0.0)
			{
				real nf = naturalFrequency(m_primitive.frequency);
				m_primitive.stiffness = springStiffness(m_a + m_b, nf);
				m_primitive.damping = springDampingCoefficient(m_a + m_b, nf, m_primitive.dampingRatio);
			}
			else
			{
				m_primitive.stiffness = 0.0;
				m_primitive.damping = 0.0;
			}
			m_primitive.gamma = constraintImpulseMixing(dt, m_primitive.stiffness, m_primitive.damping);
			real erp = errorReductionParameter(dt, m_primitive.stiffness, m_primitive.damping);

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();
			Vector2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 rb = pb - bodyB->position();

			m_primitive.bias = (pa - pb) * erp;
			Matrix3x3 k;


			m_primitive.effectiveMass = k.invert();
			m_primitive.bodyA->applyImpulse(m_primitive.accumulatedImpulse, ra);
			m_primitive.bodyB->applyImpulse(-m_primitive.accumulatedImpulse, rb);
		}
		void solveVelocity(const real& dt) override
		{

		}
		void solvePosition(const real& dt) override
		{

		}
		WeldJointPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		WeldJointPrimitive m_primitive;
	};
}
#endif