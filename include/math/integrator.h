#ifndef PHYSICS2D_INTEGRATOR_H
#define PHYSICS2D_INTEGRATOR_H
#include "include/common/common.h"
#include "include/math/linear/linear.h"
namespace Physics2D
{
	namespace Integrator
	{
		struct Timestep
		{
			real dt0 = 0;
			real dt = 0;
		};
		template<typename T = real>
		struct Value
		{
			T x0 = 0; // last time
			T x = 0;
			T dx0 = 0;
			T dx = 0;
			T ddx = 0;
			Timestep step;
		};
		
		template<typename T>
		class SemiImplicitEuler
		{
		public:
			static void integrate(Value<T>& value, real dt)
			{
				Value<T> last = value;
				value.step.dt = dt;
				value.dx += value.ddx * dt;
				value.x += value.dx * dt;

				//make it last
				value.dx0 = last.dx;
				value.x0 = last.x;
				value.step.dt0 = dt;
			}
		};
		template<typename T>
		class VerletVelocity
		{
		public:
			static void integrate(Value<T>& value, real dt)
			{
				Value<T> last = value;
				value.step.dt = dt;
				value.x = value.x + value.dx * dt + 0.5 * value.ddx * dt * dt;
				value.dx = value.dx + value.ddx * dt;

				value.x0 = last.x;
				value.dx0 = last.dx;
				value.step.dt0 = last.step.dt;
			}
		};

		template<typename T>
		class VerletPosition
		{
		public:

			static void integrate(Value<T>& value, real dt)
			{
				Value<T> last = value;
				value.step.dt = dt;
				value.x = 2 * value.x - value.x0 + value.ddx * dt * dt;

				value.x0 = last.x;
				value.dx0 = last.dx;
				value.step.dt0 = last.step.dt;
			}
		};
		template<typename T>
		class Rk4
		{
		public:
			static void integrate(Value<T>& value, real dt)
			{

			}
		};
		template<typename T>
		class Adams4
		{
		public:
			static void integrate(Value<T>& value, real dt)
			{

			}
		};
	}
}
#endif