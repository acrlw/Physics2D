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
		bool collide(const AABB& other) const;
		void scale(const number& factor);
		/// <summary>
		/// Create tight AABB from shape.
		/// </summary>
		/// <param name="shape">shape source</param>
		/// <param name="factor">AABB scale factor. Default factor means tight AABB</param>
		/// <returns></returns>
		static AABB fromShape(const ShapePrimitive& shape, const number& factor = 1);
		static bool collide(const AABB& src, const AABB& target);
		static AABB unite(const AABB& src, const AABB& target);
		static void scale(AABB& aabb, const number& factor = 1);
	};
}
#endif