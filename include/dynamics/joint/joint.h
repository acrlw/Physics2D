#ifndef PHYSICS2D_DYNAMICS_JOINT_JOINT_H
#define PHYSICS2D_DYNAMICS_JOINT_JOINT_H
#include "include/dynamics/body.h"
namespace Physics2D
{
	class Joint
	{
	public:
		Joint(){}
		virtual void prepare(const real& dt) = 0;
		virtual void solveVelocity(const real& dt) = 0;
		virtual void solvePosition(const real& dt) = 0;
	private:
	};
	
}
#endif