#ifndef PHYSICS_BROADPHASE_SAP_H
#define PHYSICS_BROADPHASE_SAP_H
#include "../../dynamics/body.h"

namespace Physics2D
{
	class SweepAndPrune
	{
	public:

		static std::vector<std::pair<Body*, Body*>> generate(const std::vector<Body*>& bodyList);
		static std::vector<Body*> query(const std::vector<Body*>& bodyList, const AABB& region);
	};
}

#endif // !PHYSICS_BROADPHASE_GRID_H
