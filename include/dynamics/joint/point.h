#ifndef PHYSICS2D_DYNAMICS_JOINT_POINT_H
#define PHYSICS2D_DYNAMICS_JOINT_POINT_H
#include "joint.h"
namespace Physics2D
{
	struct PointJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vector2 localPointA;
		Vector2 localPointB;
		real stiffness = 0.2;
		real damping = 1;
		Matrix2x2 effectiveMass;
		Vector2 bias;
		Vector2 accumulatedImpulse;
		Vector2 lastImpulse;
	};
	class PointJoint : public Joint
	{
	public:
		PointJoint() = default;
		PointJoint(const PointJointPrimitive& prim) : m_primitive(prim) {}
		void set(const PointJointPrimitive& prim)
		{
			m_primitive = prim;
		}
		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			Vector2 ra = m_primitive.localPointA;
			Vector2 rb = m_primitive.localPointB;
			Vector2 pa = m_primitive.bodyA->toWorldPoint(ra);
			Vector2 pb = m_primitive.bodyB->toWorldPoint(rb);

			real im_a = m_primitive.bodyA->inverseMass();
			real im_b = m_primitive.bodyB->inverseMass();
			real ii_a = m_primitive.bodyA->inverseInertia();
			real ii_b = m_primitive.bodyB->inverseInertia();
			real inv_dt = 1.0 / dt;
			//J M^-1  J^T
			m_primitive.effectiveMass.column1.x = im_a + im_b + ii_a * ra.y * ra.y + ii_b * rb.y * rb.y;
			m_primitive.effectiveMass.column1.y = -ii_a * ra.x * ra.y - ii_b * rb.x * rb.y;
			m_primitive.effectiveMass.column2.x = m_primitive.effectiveMass.column1.y;
			m_primitive.effectiveMass.column2.y = im_a + im_b + ii_a * ra.x * ra.x + ii_b * rb.x * rb.x;
			//(J M^-1  J^T) ^ -1
			m_primitive.effectiveMass.invert();
			//jv + b
			m_primitive.bias = -m_factor * (pb - pa) * inv_dt;
			
		}
		void solveVelocity(const real& dt) override
		{
			Vector2 ra = m_primitive.localPointA;
			Vector2 rb = m_primitive.localPointB;
			Vector2 va = m_primitive.bodyA->velocity() + Vector2::crossProduct(m_primitive.bodyA->angularVelocity(), ra);
			Vector2 vb = m_primitive.bodyB->velocity() + Vector2::crossProduct(m_primitive.bodyB->angularVelocity(), rb);
			Vector2 rv = vb - va;
			
			Vector2 impulse = m_primitive.effectiveMass.multiply(-rv + m_primitive.bias);
			//impulse += (impulse - m_primitive.lastImpulse) * m_factor;
			
			m_primitive.bodyA->applyImpulse(-impulse, ra);
			m_primitive.bodyB->applyImpulse(impulse, rb);
			
			m_primitive.accumulatedImpulse += impulse;
			m_primitive.lastImpulse = impulse;
		}
		void solvePosition(const real& dt) override
		{
			Vector2 ra = m_primitive.localPointA;
			Vector2 rb = m_primitive.localPointB;
			Vector2 pa = m_primitive.bodyA->toWorldPoint(ra);
			Vector2 pb = m_primitive.bodyB->toWorldPoint(rb);
			
			
		}
	private:
		PointJointPrimitive m_primitive;
		real m_factor = 0.2;
	};
}
#endif