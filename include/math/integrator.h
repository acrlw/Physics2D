#pragma once
#include "include/common/common.h"
#include "include/math/math.h"
namespace Physics2D
{
	struct BodyState
	{
		BodyState() = default;
		//last
		Vector2 lastPosition;
		Vector2 lastVelocity;
		number lastAngle = 0;
		number lastAngularVelocity = 0;
		number lastDeltaTime = 0;
		//now
		Vector2 position;
		Vector2 velocity;
		Vector2 acceleration;
		number angle = 0;
		number angularVelocity = 0;
		number angularAcceleration = 0;
		number deltaTime = 0;
	};
	
	class SemiImplicitEuler
	{
	public:
		static BodyState integrate(BodyState& last, const number& dt)
		{
			BodyState result;
			result.velocity = last.velocity + last.acceleration * dt;
			result.position = last.position + result.velocity * dt;
			result.angularVelocity = last.angularVelocity + last.angularAcceleration * dt;
			result.angle = last.angle + result.angularAcceleration * dt;
			result.deltaTime = dt;

			result.lastDeltaTime = dt;
			result.lastPosition = last.position;
			result.lastVelocity = last.velocity;
			result.lastAngle = last.angle;
			result.lastAngularVelocity = last.angularVelocity;
			return result;
		}
	};
	class Verlet
	{
	public:

		static BodyState integrateVelocity(BodyState& state, const number& dt)
		{
			BodyState result;
			Vector2 lastVelocity = state.position - state.lastPosition;
			result.velocity = lastVelocity + state.acceleration * dt * dt;
			result.position = state.position + result.velocity;
			result.angularVelocity = (state.angle - state.lastAngle) + state.angularVelocity * dt * dt;
			result.angle = state.angle + result.angularVelocity;

			result.lastDeltaTime = dt;
			result.lastAngle = state.angle;
			result.lastPosition = state.position;
			result.lastVelocity = state.velocity;
			result.lastAngularVelocity = state.angularVelocity;
			
			return result;
		}
		static BodyState integratePosition(BodyState& state, const number& dt)
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
