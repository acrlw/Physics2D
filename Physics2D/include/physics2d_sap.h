#ifndef PHYSICS_BROADPHASE_SAP_H
#define PHYSICS_BROADPHASE_SAP_H
#include "physics2d_body.h"

namespace Physics2D
{
	class PHYSICS2D_API SweepAndPrune
	{
	public:

		static Container::Vector<std::pair<Body*, Body*>> generate(const Container::Vector<Body*>& bodyList);
		static Container::Vector<Body*> query(const Container::Vector<Body*>& bodyList, const AABB& region);
	};
}

#endif // !PHYSICS_BROADPHASE_GRID_H
