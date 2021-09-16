#ifndef PHYSICS2D_DYNAMICS_JOINT_JOINT_H
#define PHYSICS2D_DYNAMICS_JOINT_JOINT_H
#include "include/dynamics/body.h"
namespace Physics2D
{
	enum class JointType
	{
		Distance,
		Point,
		Rotation,
		Pulley,
		Prismatic,
		Wheel,
		Revolute,
		Mouse,
	};
	class Joint
	{
	public:
		Joint(){}
		virtual void prepare(const real& dt) = 0;
		virtual Vector2 solveVelocity(const real& dt) = 0;
		virtual void solvePosition(const real& dt) = 0;
		JointType type()const
		{
			return m_type;
		}
	protected:
		JointType m_type;
	};
	
}
#endif