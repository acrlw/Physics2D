#ifndef PHYSICS_BROADPHASE_GRID_H
#define PHYSICS_BROADPHASE_GRID_H
#include "include/collision/broadphase/aabb.h"
namespace Physics2D
{
	class UniformGrid
	{
	public:
		std::vector<std::pair<Body*, Body*>> generatePairs();
		void update(Body* body);
		void insert(Body* body);
	private:
		real m_gridSize = 1;
		
	};
}

#endif // !PHYSICS_BROADPHASE_GRID_H
#define PHYSICS_BROADPHASE_GRID_H
