#include "include/collision/broadphase/aabb.h"

namespace Physics2D
{
	AABB AABB::fromShape(const ShapePrimitive& shape)
	{
		return AABB();
	}
	bool AABB::collide(const AABB& src, const AABB& target)
	{
		return false;
	}
	AABB AABB::unite(const AABB& src, const AABB& target)
	{
		return AABB();
	}
}
