#pragma once
#include "include/common/common.h"
#include "include/math/math.h"
namespace Physics2D
{
	struct BodyState
	{
		Vector2 lastPosition;
		Vector2 lastVelocity;
		number lastAngle;
		number lastAngularVelocity;
		number lastDeltaTime;
	};
	class Integrator
	{
	public:
		virtual void integrate() = 0;
	};
	class SemiImplicitEuler : public Integrator
	{
	public:
		virtual void integrate() override
		{
			
		}
	};
	class Verlet : public Integrator
	{
	public:
		virtual void integrate() override
		{

		}
	};
	class Rk4 : public Integrator
	{
	public:
		virtual void integrate() override
		{

		}
	};
}
