#ifndef PHYSICS_RANDOM_H
#define PHYSICS_RANDOM_H

#include "random"

namespace Physics2D
{

	static std::random_device randomDevice;
	static std::mt19937_64 randomEngine(randomDevice());
	static std::uniform_int_distribution<int16_t> uniformDistribution;

	class RandomGenerator
	{
	public:
		static int unique()
		{
			return uniformDistribution(randomEngine);
		}
	};
}

#endif