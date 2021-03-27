#pragma once
#include "include/math/math.h"
#include "include/common/common.h"
#include "include/dynamics/shape.h"
namespace Physics2D
{
	class RigidBody
	{
	public:
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
		Vector2 m_acceleration;
		number m_angle;
		number m_angularVelocity;
		number m_angularAcceleration;
		Vector2 m_forces;
		number m_torques = 0;
		Shape* m_shape = nullptr;
	};
}