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
		real maxForce = 20000;
	};
	class PointJoint : public Joint
	{
	public:
		PointJoint()
		{
			m_type = JointType::Point;
		}
		PointJoint(const PointJointPrimitive& prim) : m_primitive(prim)
		{
			m_type = JointType::Point;
		}
		void set(const PointJointPrimitive& prim)
		{
			m_primitive = prim;
		}
		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			
		}
		void solveVelocity(const real& dt) override
		{

		}
		void solvePosition(const real& dt) override
		{
			
			
		}
		PointJointPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		PointJointPrimitive m_primitive;
		real m_factor = 0.22;
	};
}
#endif