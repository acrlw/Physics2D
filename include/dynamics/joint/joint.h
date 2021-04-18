#ifndef PHYSICS2D_DYNAMICS_JOINT_JOINT_H
#define PHYSICS2D_DYNAMICS_JOINT_JOINT_H
#include "include/dynamics/body.h"
namespace Physics2D
{
	struct Joint
	{
		Joint() :
			body1(nullptr), body2(nullptr),
			biasFactor(0.2f), softness(0.0f)
		{}

		void set(Body* b1, Body* b2, const Vector2& anchor)
		{
			body1 = b1;
			body2 = b2;

			Matrix2x2 Rot1(body1->angle());
			Matrix2x2 Rot2(body2->angle());
			Matrix2x2 Rot1T = Rot1.transpose();
			Matrix2x2 Rot2T = Rot2.transpose();

			localAnchor1 = Rot1T.multiply(anchor - body1->position());
			localAnchor2 = Rot2T.multiply(anchor - body2->position());

			P.clear();

			softness = 0.0f;
			biasFactor = 0.2f;
		}

		void preStep(float inv_dt)
		{
			// Pre-compute anchors, mass matrix, and bias.
			Matrix2x2 Rot1(body1->angle());
			Matrix2x2 Rot2(body2->angle());

			r1 = Rot1.multiply(localAnchor1);
			r2 = Rot2.multiply(localAnchor2);

			// deltaV = deltaV0 + K * impulse
			// invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
			//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
			//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
			Matrix2x2 K1;
			K1.column1.x = body1->inverseMass() + body2->inverseMass();	K1.column2.x = 0.0f;
			K1.column1.y = 0.0f;								K1.column2.y = body1->inverseMass() + body2->inverseMass();

			Matrix2x2 K2;
			K2.column1.x = body1->inverseInertia() * r1.y * r1.y;		K2.column2.x = -body1->inverseInertia() * r1.x * r1.y;
			K2.column1.y = -body1->inverseInertia() * r1.x * r1.y;		K2.column2.y = body1->inverseInertia() * r1.x * r1.x;

			Matrix2x2 K3;
			K3.column1.x = body2->inverseInertia() * r2.y * r2.y;		K3.column2.x = -body2->inverseInertia() * r2.x * r2.y;
			K3.column1.y = -body2->inverseInertia() * r2.x * r2.y;		K3.column2.y = body2->inverseInertia() * r2.x * r2.x;

			Matrix2x2 K = K1 + K2 + K3;
			K.column1.x += softness;
			K.column2.y += softness;

			M = K.invert();

			Vector2 p1 = body1->position() + r1;
			Vector2 p2 = body2->position() + r2;
			Vector2 dp = p2 - p1;
			
		    bias = -biasFactor * inv_dt * dp;
			
			// Apply accumulated impulse.
			body1->velocity() -= body1->inverseMass() * P;
			body1->angularVelocity() -= body1->inverseInertia() * r1.cross(P);

			body2->velocity() += body2->inverseMass() * P;
			body2->angularVelocity() += body2->inverseInertia() * r2.cross(P);
			
		}
		void applyImpulse()
		{
			Vector2 dv = body2->velocity() + Vector2::crossProduct(body2->angularVelocity(), r2) - body1->velocity() - Vector2::crossProduct(body1->angularVelocity(), r1);

			Vector2 impulse;

			impulse = M.multiply(bias - dv - softness * P);

			body1->velocity() -= body1->inverseMass() * impulse;
			body1->angularVelocity() -= body1->inverseInertia() * r1.cross(impulse);

			body2->velocity() += body2->inverseMass() * impulse;
			body2->angularVelocity() += body2->inverseInertia() * r2.cross(impulse);

			P += impulse;
		}

		Matrix2x2 M;
		Vector2 localAnchor1, localAnchor2;
		Vector2 r1, r2;
		Vector2 bias;
		Vector2 P;		// accumulated impulse
		Body* body1;
		Body* body2;
		float biasFactor;
		float softness;
	};
	
}
#endif