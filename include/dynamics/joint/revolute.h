#ifndef PHYSICS2D_DYNAMICS_JOINT_REVOLUTE_H
#define PHYSICS2D_DYNAMICS_JOINT_REVOLUTE_H

namespace Physics2D
{
	struct RevoluteJointPrimitive
	{
		
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

		}
		void solveVelocity(const real& dt) override
		{

		}
		void solvePosition(const real& dt) override
		{

		}
	private:
		RevoluteJointPrimitive m_primitive;
	};
}
#endif