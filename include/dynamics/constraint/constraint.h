#ifndef PHYSICS2D_DYNAMICS_CONSTRAINT_H
#define PHYSICS2D_DYNAMICS_CONSTRAINT_H
#include "include/dynamics/body.h"
namespace Physics2D
{
	struct DistanceConstraintPrimitive
	{
		Body* source;
		Vector2 sourcePoint;
		Vector2 targetPoint;
		real distance = 0;
		Vector2 lastError;
		real lastCross = 0;
		real allCross = 0;
		Vector2 impulse;
		Vector2 lastImpulse;
		real damping = 0.4;
		real maxForce = 8000;
	};
	
	class DistanceConstraintSolver
	{
	public:
		void solve(DistanceConstraintPrimitive& primitive, const real& dt)
		{
			if (primitive.source == nullptr)
				return;
			
		}
	private:
		real beta = 0.5;
	};
}
#endif