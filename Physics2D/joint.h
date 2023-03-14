#ifndef PHYSICS2D_DYNAMICS_JOINT_JOINT_H
#define PHYSICS2D_DYNAMICS_JOINT_JOINT_H
#include "body.h"
namespace Physics2D
{
	enum class JointType
	{
		Distance,
		Point,
		Rotation,
		Orientation,
		Pulley,
		Prismatic,
		Weld,
		Wheel,
		Revolute
	};
	class Joint
	{
	public:
		Joint(){}
		virtual void prepare(const real& dt) = 0;
		virtual void solveVelocity(const real& dt) = 0;
		virtual void solvePosition(const real& dt) = 0;
		bool active()
		{
			return m_active;
		}
		void setActive(bool active)
		{
			m_active = active;
		}
		JointType type()const
		{
			return m_type;
		}
		uint32_t id()const
		{
			return m_id;
		}
		void setId(const uint32_t& id)
		{
			m_id = id;
		}
		static real naturalFrequency(real frequency)
		{
			return Constant::DoublePi * frequency;
		}
		static real springDampingCoefficient(real mass, real naturalFrequency, real dampingRatio)
		{
			return dampingRatio * 2.0f * mass * naturalFrequency;
		}
		static real springStiffness(real mass, real naturalFrequency)
		{
			return mass * naturalFrequency * naturalFrequency;
		}
		static real constraintImpulseMixing(real dt, real stiffness, real damping)
		{
			real cim = dt * (dt * stiffness + damping);
			return realEqual(cim, 0.0f) ? 0.0f : 1.0f / cim;
		}
		static real errorReductionParameter(real dt, real stiffness, real damping)
		{
			real erp = dt * stiffness + damping;
			return realEqual(erp, 0.0f) ? 0.0f : stiffness / erp;
		}
	protected:
		bool m_active = true;
		JointType m_type;
		uint32_t m_id;
	};
	
}
#endif