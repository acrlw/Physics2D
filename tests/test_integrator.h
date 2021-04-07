#pragma once
#include "test.h"
#include "include/physics2d.h"

namespace Physics2D
{
	class IntegratorTest: public Test
	{
		IntegratorTest()
		{
			m_name = "integrator test";
		}
		void run() override
		{
			
		}
		void testVerlet()
		{
			BodyState state;
			state.position.set(0, 20);
			state.acceleration.set(0, 9.8f);
			state.angularVelocity = 20;
			state.angle = 15;
			state.deltaTime = 1.0f / 60.0f;
			state.lastPosition.set(0, 20);
			state.lastDeltaTime = 1.0f / 60.0f;
			state.lastAngle = 15;
			for (size_t i = 0; i < 100; i += 1)
			{
				state.acceleration.set(0, 9.8f);
				state.angularAcceleration = -8;
				//fmt::print("----step{}-----\n", i);
				fmt::print("position:{}\n", state.position);
				//fmt::print("velocity:{}\n", state.velocity);
				//fmt::print("acceleration:{}\n", state.acceleration);
				//fmt::print("angle:{}\n", state.angle);
				//fmt::print("angularVelocity:{}\n", state.angularVelocity);
				//fmt::print("--------------\n");
				state = Verlet::integrateVelocity(state, 1.0f / 60.0f);
			}
		}
	};
}
