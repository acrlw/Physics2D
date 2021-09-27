#ifndef PHYSICS_RANDOM_H
#define PHYSICS_RANDOM_H

#include "random"

namespace Physics2D
{

	static std::random_device randomDevice;
	static std::mt19937_64 randomEngine(randomDevice());
	static std::uniform_int_distribution<uint16_t> uniformDistribution(10000, 99999);

	class RandomGenerator
	{
	public:
		static uint16_t unique()
		{
			return uniformDistribution(randomEngine);
		}
	};
}

#endif