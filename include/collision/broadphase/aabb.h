#ifndef PHYSICS2D_BROADPHASE_AABB_H
#define PHYSICS2D_BROADPHASE_AABB_H

#include "include/math/linear/linear.h"
#include "include/dynamics/shape.h"
namespace Physics2D
{
	struct AABB
	{
		AABB() = default;
		number width;
		number height;
		Vector2 position;
		bool collide(const AABB& other) const
		{
			return AABB::collide(*this, other);
		}
		static AABB fromShape(const ShapePrimitive& shape);
		static bool collide(const AABB& src, const AABB& target);
		static AABB unite(const AABB& src, const AABB& target);
	};
}
#endif