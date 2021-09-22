#ifndef PHYSICS2D_DYNAMICS_JOINT_MOUSE_H
#define PHYSICS2D_DYNAMICS_JOINT_MOUSE_H
namespace Physics2D
{
	struct MouseJointPrimitive
	{
		Body* bodyA;
		Vector2 localPointA;
		Vector2 mousePoint;
		Vector2 normal;
		real damping = 0;
		real stiffness = 1;
		real bias = 0;
		real biasFactor = 0.05;
		real effectiveMass = 0;
		real accumulatedImpulse = 0;
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

			
		}
		void solveVelocity(const real& dt) override
		{
			
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