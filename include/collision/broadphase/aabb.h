#ifndef PHYSICS2D_BROADPHASE_AABB_H
#define PHYSICS2D_BROADPHASE_AABB_H

#include "include/math/linear/linear.h"
namespace Physics2D
{
	struct AABB
	{
		AABB() = default;
		number width;
		number height;
		Vector2 position;
		bool collide(const AABB& other)
		{
			return AABB::collide(*this, other);
		}
		static bool collide(const AABB& src, const AABB& target)
		{
			
		}
		static AABB unite(const AABB& src, const AABB& target)
		{
			
		}
	};
}
#endif