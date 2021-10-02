#ifndef PHYSICS2D_DYNAMICS_JOINT_REVOLUTE_H
#define PHYSICS2D_DYNAMICS_JOINT_REVOLUTE_H

namespace Physics2D
{
	struct RevoluteJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vector2 localPointA;
		Vector2 localPointB;

		real damping = 0.0;
		real stiffness = 0.0;
		real frequency = 8;
		real maxForce = 200;
		real dampingRatio = 0.2;
		real gamma = 0.0;
		Vector2 bias;
		Matrix2x2 effectiveMass;
		Vector2 impulse;
	};
	class RevoluteJoint : public Joint
	{
	public:
		RevoluteJoint()
		{
			m_type = JointType::Revolute;
		}
		RevoluteJoint(const RevoluteJointPrimitive& primitive)
		{
			m_type = JointType::Revolute;
			m_primitive = primitive;
		}
		void set(const RevoluteJointPrimitive& primitive)
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
				m_primitive.stiffness = springStiffness(m_a, nf);
				m_primitive.damping = springDampingCoefficient(m_a, nf, m_primitive.dampingRatio);
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
			Vector2 pb = bodyA->toWorldPoint(m_primitive.localPointB);
			Vector2 rb = pb - bodyB->position();

			m_primitive.bias = (pa - pb) * erp;
			Matrix2x2 k;
			k.e11() = im_a + ra.y * ra.y * ii_a + im_b + rb.y * rb.y * ii_b;
			k.e12() = -ra.x * ra.y * ii_a - rb.x * rb.y * ii_b;
			k.e21() = k.e12();
			k.e22() = im_a + ra.x * ra.x * ii_a + im_b + rb.x * rb.x * ii_b;

			k.e11() += m_primitive.gamma;
			k.e22() += m_primitive.gamma;

			m_primitive.effectiveMass = k.invert();

		}
		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			Vector2 ra = m_primitive.bodyA->toWorldPoint(m_primitive.localPointA) - m_primitive.bodyA->position();
			Vector2 va = m_primitive.bodyA->velocity() + Vector2::crossProduct(m_primitive.bodyA->angularVelocity(), ra);
			Vector2 rb = m_primitive.bodyB->toWorldPoint(m_primitive.localPointB) - m_primitive.bodyB->position();
			Vector2 vb = m_primitive.bodyB->velocity() + Vector2::crossProduct(m_primitive.bodyB->angularVelocity(), rb);

			Vector2 jvb = va - vb;
			jvb += m_primitive.bias;
			jvb += m_primitive.impulse * m_primitive.gamma;
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
			m_primitive.bodyA->applyImpulse(J, ra);
			m_primitive.bodyB->applyImpulse(-J, rb);

		}
		void solvePosition(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

		}
	private:
		RevoluteJointPrimitive m_primitive;
	};
}
#endif