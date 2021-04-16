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
		real distance;
	};
	class DistanceConstraintSolver
	{
	public:
		void solve(DistanceConstraintPrimitive& primitive, const real& dt)
		{
			if (primitive.source == nullptr)
				return;

			Vector2 p = primitive.source->velocity() + Vector2::crossProduct(primitive.source->angularVelocity(), primitive.sourcePoint);
			Vector2 n = (primitive.targetPoint - primitive.source->position()).normal();
			
			real effectiveMass = primitive.source->inverseMass() + primitive.source->inverseInertia() * Vector2::crossProduct(n, primitive.sourcePoint);

		}
	};
}
#endif