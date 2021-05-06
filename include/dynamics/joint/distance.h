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
		Vector2 accumulatedImpulse;
		real maxForce = 20000;
		real c = 0;
	};
	class DistanceJoint : public Joint
	{
	public:
		DistanceJoint()
		{
			m_type = JointType::Distance;
		}
		DistanceJoint(const DistanceJointPrimitive& primitive) : m_primitive(primitive)
		{
			m_type = JointType::Distance;
		}
		void set(const DistanceJointPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{
			assert(m_primitive.minDistance <= m_primitive.maxDistance);
			
		}
		Vector2 solveVelocity(const real& dt) override
		{
			return Vector2();
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
		real m_factor = 0.6;
	};
}
#endif