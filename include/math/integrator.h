#ifndef PHYSICS2D_INTEGRATOR_H
#define PHYSICS2D_INTEGRATOR_H
#include "include/common/common.h"
#include "include/math/linear/linear.h"
namespace Physics2D
{
	struct BodyState
	{
		BodyState() = default;
		//last
		Vector2 lastPosition;
		Vector2 lastVelocity;
		real lastAngle = 0;
		real lastAngularVelocity = 0;
		real lastDeltaTime = 0;
		//now
		Vector2 position;
		Vector2 velocity;
		Vector2 acceleration;
		real angle = 0;
		real angularVelocity = 0;
		real angularAcceleration = 0;
		real deltaTime = 0;
	};
	class Integrator
	{
	public:
		enum class Type
		{
			SemiImplicitEuler,
			VerletVelocity,
			VerletPosition
		};
		virtual BodyState integrate(BodyState& last, const real& dt)
		{
			return BodyState();
		};
	protected:
		Type m_type;
	};
	
	class SemiImplicitEuler : public Integrator
	{
	public:
		SemiImplicitEuler()
		{
			m_type = Type::SemiImplicitEuler;
		}
		BodyState integrate(BodyState& last, const real& dt)override
		{
			BodyState result;
			result.velocity = last.velocity + last.acceleration * dt;
			result.position = last.position + result.velocity * dt;
			result.angularVelocity = last.angularVelocity + last.angularAcceleration * dt;
			result.angle = last.angle + result.angularVelocity * dt;
			result.deltaTime = dt;

			result.lastDeltaTime = dt;
			result.lastPosition = last.position;
			result.lastVelocity = last.velocity;
			result.lastAngle = last.angle;
			result.lastAngularVelocity = last.angularVelocity;
			return result;
		}
	};
	class VerletVelocity : public Integrator
	{
	public:

		VerletVelocity()
		{
			m_type = Type::VerletVelocity;
		}
		BodyState integrate(BodyState& state, const real& dt)override
		{
			BodyState result;
			Vector2 lastVelocity = state.position - state.lastPosition;
			result.velocity = lastVelocity + state.acceleration * dt * dt;
			result.position = state.position + result.velocity;
			result.angularVelocity = (state.angle - state.lastAngle) + state.angularAcceleration * dt * dt;
			result.angle = state.angle + result.angularVelocity;

			result.lastDeltaTime = dt;
			result.lastAngle = state.angle;
			result.lastPosition = state.position;
			result.lastVelocity = state.velocity;
			result.lastAngularVelocity = state.angularVelocity;
			
			return result;
		}
	};

	class VerletPosition : public Integrator
	{
	public:

		VerletPosition()
		{
			m_type = Type::VerletPosition;
		}
		BodyState integrate(BodyState& state, const real& dt)override
		{
			BodyState result;
			result.position = state.position + ((state.position - state.lastPosition) * dt / state.lastDeltaTime) +
				state.acceleration * dt * dt;
			result.angle = state.angle + ((state.angle - state.lastAngle) * dt / state.lastDeltaTime) +
				state.angularAcceleration * dt * dt;

			result.velocity = (result.position - state.lastPosition) / (2 * dt);
			result.angularAcceleration = (result.angle - state.lastAngle) / (2 * dt);

			result.lastDeltaTime = dt;
			result.lastPosition = state.position;
			result.lastAngle = state.angle;
			result.lastVelocity = state.velocity;
			result.lastAngularVelocity = state.angularVelocity;

			return result;
		}
	};
	class Rk4
	{
	public:
		
	};
}
#endif