#ifndef PHYSICS_RANDOM_H
#define PHYSICS_RANDOM_H

#include "QtCore/QRandomGenerator"

namespace Physics2D
{

	static std::random_device randomDevice;
	static std::mt19937_64 randomEngine(randomDevice());
	static std::uniform_int_distribution<int16_t> uniformDistribution;

	class RandomGenerator
	{
	public:
		static int generate(int min, int max)
		{
			return QRandomGenerator::global()->bounded(min, max);
		}
		static int unique()
		{
			return uniformDistribution(randomEngine);
		}
		static bool pop(size_t number)
		{
			for(auto iter = m_uniqueList.begin(); iter != m_uniqueList.end();++iter)
			{
				if((*iter) == number)
				{
					m_uniqueList.erase(iter);
					return true;
				}
			}
			return false;
		}
		static std::vector<int> m_uniqueList;
	};
}

#endif