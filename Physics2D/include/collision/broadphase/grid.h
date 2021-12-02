#ifndef PHYSICS_BROADPHASE_GRID_H
#define PHYSICS_BROADPHASE_GRID_H
#include "../../collision/broadphase/aabb.h"
namespace Physics2D
{
	class UniformGrid
	{
	public:
		std::vector<std::pair<Body*, Body*>> generate();
		std::vector<Body*> raycast(const Vector2& p, const Vector2& d);
		void update(Body* body);
		void insert(Body* body);
		void remove(Body* body);
	private:
		real m_gridSize = 1;
		
	};
}

#endif // !PHYSICS_BROADPHASE_GRID_H
#define PHYSICS_BROADPHASE_GRID_H
