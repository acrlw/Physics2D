#ifndef PHYSICS2D_DYNAMICS_JOINT_MOUSE_H
#define PHYSICS2D_DYNAMICS_JOINT_MOUSE_H
namespace Physics2D
{
	struct MouseJointPrimitive
	{
		Body* bodyA;
		Vector2 localPointA;
		Vector2 mousePoint;
		Matrix2x2 effectiveMass;
		real damping = 0;
		real stiffness = 1;
		Vector2 bias;
		Vector2 lastImpulse;
		Vector2 accumulatedImpulse;
		real maxForce = 20000;
	};
	class MouseJoint : public Joint
	{
	public:
		MouseJoint()
		{
			m_type = JointType::Mouse;
		}
		MouseJoint(const MouseJointPrimitive& primitive)
		{
			m_type = JointType::Mouse;
			m_primitive = primitive;
		}
		void set(const MouseJointPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		MouseJointPrimitive primitive()const
		{
			return m_primitive;
		}
		void prepare(const real& dt) override
		{
			Vector2 ra = Matrix2x2(m_primitive.bodyA->angle()).multiply(m_primitive.localPointA);
			Vector2 c = m_primitive.mousePoint - (m_primitive.bodyA->position() + ra);
			Matrix2x2 k;

			real im_a = m_primitive.bodyA->inverseMass();
			real ii_a = m_primitive.bodyA->inverseInertia();
			real inv_dt = 1.0 / dt;
			
			k.e11() = im_a + ii_a * ra.y * ra.y;
			k.e21() = -ii_a * ra.x * ra.y;
			k.e12() = -ii_a * ra.x * ra.y;
			k.e22() = im_a + ii_a * ra.x * ra.x;

			m_primitive.effectiveMass = k.invert();
			m_primitive.bias = - c * inv_dt;
		}
		Vector2 solveVelocity(const real& dt) override
		{
			Vector2 ra = Matrix2x2(m_primitive.bodyA->angle()).multiply(m_primitive.localPointA);
			Vector2 va = m_primitive.bodyA->velocity() + Vector2::crossProduct(m_primitive.bodyA->angularVelocity(), ra);
			
			Vector2 jv = va + m_primitive.bias;
			Vector2 impulse = m_primitive.effectiveMass.multiply(jv.negate());
			impulse += (impulse - m_primitive.lastImpulse) * dt;
			m_primitive.bodyA->applyImpulse(impulse, ra);
			
			m_primitive.lastImpulse = impulse;
			return impulse;
		}
		void solvePosition(const real& dt) override
		{

		}
	private:
		MouseJointPrimitive m_primitive;
		real m_factor = 0.5;
	};
}
#endif