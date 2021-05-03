#ifndef PHYSICS2D_DYNAMICS_JOINT_DISTANCE_H
#define PHYSICS2D_DYNAMICS_JOINT_DISTANCE_H
#include "joint.h"
namespace Physics2D
{
	struct DistanceJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vector2 localPointA;
		Vector2 localPointB;
		real maxDistance = 0;
		real minDistance = 0;
		real stiffness = 1;
		real damping = 1;
		real effectiveMass = 0;
		Vector2 bias;
	};
	class DistanceJoint : public Joint
	{
	public:
		DistanceJoint()
		{
			m_type = JointType::Distance;
		}
		void set(const DistanceJointPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{
			Vector2 ra = m_primitive.localPointA;
			Vector2 rb = m_primitive.localPointB;
			
			Vector2 pa = m_primitive.bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 pb = m_primitive.bodyB->toWorldPoint(m_primitive.localPointB);

			real im_a = m_primitive.bodyA->inverseMass();
			real im_b = m_primitive.bodyB->inverseMass();
			real ii_a = m_primitive.bodyA->inverseInertia();
			real ii_b = m_primitive.bodyB->inverseInertia();
			
			real length = (pa - pb).length();
			real c = 0;
			if (length < m_primitive.minDistance)
			{
				//push two body to prevent
				c = m_primitive.minDistance - length;
			}
			else if (length > m_primitive.maxDistance)
			{
				//pull two body to prevent exceed max
				c = m_primitive.maxDistance - length;
			}
			else
				return;
			Vector2 n = (pa - pb).normal();
			
			real n_ra = n.cross(ra);
			real n_rb = n.cross(rb);
			
			real inv_dt = 1.0 / dt;
			m_primitive.effectiveMass = 1.0 /
				(im_a + im_b + ii_a * n_ra * n_ra + ii_b * n_rb * n_rb);

			m_primitive.bias = -m_factor * inv_dt * c * n;
		}
		void solveVelocity(const real& dt) override
		{
			Vector2 ra = m_primitive.localPointA;
			Vector2 rb = m_primitive.localPointB;
			
			Vector2 pa = m_primitive.bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 pb = m_primitive.bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 n = (pa - pb).normal();

			Vector2 va = m_primitive.bodyA->velocity() + Vector2::crossProduct(m_primitive.bodyA->angularVelocity(), ra);
			Vector2 vb = m_primitive.bodyB->velocity() + Vector2::crossProduct(m_primitive.bodyB->angularVelocity(), rb);
			Vector2 rv = vb - va;

			Vector2 impulse = m_primitive.effectiveMass * (-rv + m_primitive.bias);
			m_primitive.bodyA->applyImpulse(-impulse, ra);
			m_primitive.bodyB->applyImpulse(impulse, rb);


			
		}
		void solvePosition(const real& dt) override
		{
			
		}

		DistanceJointPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		DistanceJointPrimitive m_primitive;
		real m_factor = 0.2;
	};
}
#endif