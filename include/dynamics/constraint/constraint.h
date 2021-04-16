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
	};
	
	class DistanceConstraintSolver
	{
	public:
		void solve(DistanceConstraintPrimitive& primitive, const real& dt)
		{
			if (primitive.source == nullptr)
				return;
			for(int i = 0;i < 1;i++)
			{
				Vector2 ra = Matrix2x2(primitive.source->angle()).multiply(primitive.sourcePoint);
				Vector2 pa = ra + primitive.source->position();
				Vector2 pb = primitive.targetPoint;
				Vector2 crWr = Vector2::crossProduct(primitive.source->angularVelocity(), ra);
				Vector2 vel = primitive.source->velocity();


				Vector2 n = (pb - pa).normal();

				Vector2 tb = (n * primitive.distance).negate() + primitive.targetPoint;
				Vector2 cPos = tb - pa;
				const real effectiveMass = 1.0f / (primitive.source->inverseMass() + primitive.source->inverseInertia() * pow(ra.cross(n), 2));
				//const real effectiveMass = 1.0f / (primitive.source->inverseMass());
				real lambda = -effectiveMass * (vel.dot(n));
				Vector2 v = primitive.source->inverseMass() * lambda * n;
				v += cPos * beta + (cPos - primitive.lastError) * beta;
				real torque = ra.cross(n) * 15 * beta;
				if(primitive.lastError != 0)
					torque += (torque - primitive.lastCross) * beta * dt;

				primitive.source->velocity() += v;
				primitive.source->angularVelocity() += primitive.source->inverseInertia() * torque;
				primitive.lastError = cPos;
				primitive.lastCross = torque;
				primitive.allCross += torque;
			}
		}
	private:
		real beta = 0.8;
	};
}
#endif