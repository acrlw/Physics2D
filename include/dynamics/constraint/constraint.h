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
		real stiffness = 0.5;
	};
	
	class DistanceConstraintSolver
	{
	public:
		void solve(const real& dt)
		{
			for(DistanceConstraintPrimitive& primitive: m_list)
			{
				if (primitive.source == nullptr)
					continue;
				for (int i = 0; i < 20; i++)
				{
					Vector2 ra = Matrix2x2(primitive.source->angle()).multiply(primitive.sourcePoint);
					Vector2 pa = ra + primitive.source->position();
					Vector2 pb = primitive.targetPoint;
					Vector2 crWr = Vector2::crossProduct(primitive.source->angularVelocity(), ra);
					Vector2 vel = primitive.source->velocity();


					Vector2 n = (pb - pa).normal();

					Vector2 tb = (n * primitive.distance).negate() + primitive.targetPoint;
					Vector2 cPos = tb - pa;
					const real effectiveMass = 1.0 / (primitive.source->inverseMass() + primitive.source->inverseInertia() * pow(ra.cross(n), 2));
					real lambda = -effectiveMass * (vel.dot(n));
					Vector2 v = primitive.source->inverseMass() * lambda * n;
					v += cPos * primitive.stiffness + (cPos - primitive.lastError) * primitive.stiffness;
					real torque = ra.cross(n);
					if (primitive.lastError != 0)
						torque += (torque - primitive.lastCross) * primitive.stiffness * dt;

					primitive.source->velocity() += v;
					primitive.source->angularVelocity() += primitive.source->inverseInertia() * torque;
					primitive.lastError = cPos;
					primitive.lastCross = torque;

					//angle and angular velocity approximation
					//if (abs(primitive.source->angle()) < 0.5)
					//{
					//	if (abs(primitive.source->angularVelocity()) < 0.5)
					//	{
					//		primitive.source->angularVelocity() = 0;
					//	}
					//	primitive.source->angle() = 0;
					//}
				}
			}
		}
		void add(const DistanceConstraintPrimitive& primitive)
		{
			m_list.emplace_back(primitive);
		}
	private:
		std::vector<DistanceConstraintPrimitive> m_list;
	};
}
#endif