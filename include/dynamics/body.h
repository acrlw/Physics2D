#ifndef PHYSICS2D_BODY_H
#define PHYSICS2D_BODY_H
#include "include/math/math.h"
#include "include/common/common.h"
#include "include/dynamics/shape.h"
namespace Physics2D
{
	class Body
	{
	public:
		enum class BodyType
		{
			Kinematic,
			Static,
			Dynamic,
			Bullet
		};
		Vector2 position()const
		{
			return m_position;
		}
		number angle()const
		{
			return m_angle;
		}
		Shape* shape()const
		{
			return m_shape;
		}
	private:
		Vector2 m_position;
		Vector2 m_velocity;
		number m_angle;
		number m_angularVelocity;
		Vector2 m_forces;
		number m_torques = 0;
		Shape* m_shape = nullptr;
		BodyType m_type;
	};
}
#endif