#ifndef PHYSICS2D_DYNAMICS_JOINT_PULLEY_H
#define PHYSICS2D_DYNAMICS_JOINT_PULLEY_H

namespace Physics2D
{
	struct PulleyJointPrimitive
	{
		
	};
	class PulleyJoint : public Joint
	{
	public:
		PulleyJoint()
		{
			m_type = JointType::Pulley;
		}

		PulleyJoint(const PulleyJointPrimitive& primitive)
		{
			m_type = JointType::Mouse;
			m_primitive = primitive;
		}
		void set(const PulleyJointPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{

		}
		Vector2 solveVelocity(const real& dt) override
		{
			return Vector2();
		}
		void solvePosition(const real& dt) override
		{

		}
	private:
		PulleyJointPrimitive m_primitive;
	};
}
#endif