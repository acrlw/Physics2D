#ifndef PHYSICS_RANDOM_H
#define PHYSICS_RANDOM_H

#include "QtCore/QRandomGenerator"
namespace Physics2D
{
	class RandomGenerator
	{
	public:
		static int generate(int min, int max)
		{
			return QRandomGenerator::global()->bounded(min, max);
		}
		static int unique(int min, int max)
		{
			while(true)
			{
				int result = generate(min, max);
				if (std::find(std::begin(m_uniqueList), std::end(m_uniqueList), result) == std::end(m_uniqueList))
				{
					m_uniqueList.push_back(result);
					return result;
				}
			}
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