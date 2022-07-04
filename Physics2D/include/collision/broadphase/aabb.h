#ifndef PHYSICS2D_BROADPHASE_AABB_H
#define PHYSICS2D_BROADPHASE_AABB_H

#include "../../math/linear/linear.h"
#include "../../geometry/shape.h"

namespace Physics2D
{
	class Body;

	struct AABB
	{
		AABB() = default;
		AABB(const Vector2& topLeft, const real& boxWidth, const real& boxHeight);
		AABB(const Vector2& topLeft, const Vector2& bottomRight);
		real width = 0;
		real height = 0;
		Vector2 position;
		inline Vector2 topLeft()const;
		inline Vector2 topRight()const;
		inline Vector2 bottomLeft()const;
		inline Vector2 bottomRight()const;

		inline real minimumX()const;
		inline real minimumY()const;
		inline real maximumX()const;
		inline real maximumY()const;

		bool collide(const AABB& other) const;
		void expand(const real& factor);
		void scale(const real& factor);
		void clear();
		AABB& unite(const AABB& other);
		real surfaceArea()const;
		real volume()const;
		bool isSubset(const AABB& other)const;
		bool isEmpty()const;
		bool operator==(const AABB& other)const;
		bool raycast(const Vector2& start, const Vector2& direction)const;
		/// <summary>
		/// Create AABB from shape.
		/// </summary>
		/// <param name="shape">shape source</param>
		/// <param name="factor">AABB scale factor. Default factor 1 means making tight AABB</param>
		/// <returns></returns>
		static AABB fromShape(const ShapePrimitive& shape, const real& factor = 0);
		static AABB fromBody(Body* body, const real& factor = 0);
		static AABB fromBox(const Vector2& topLeft, const Vector2& bottomRight);
		/// <summary>
		/// Check if two aabbs are overlapping
		/// </summary>
		/// <param name="src"></param>
		/// <param name="target"></param>
		/// <returns></returns>
		static bool collide(const AABB& src, const AABB& target);
		/// <summary>
		/// Return two aabb union result
		/// </summary>
		/// <param name="src"></param>
		/// <param name="target"></param>
		/// <returns></returns>
		static AABB unite(const AABB& src, const AABB& target, const real& factor = 0);
		/// <summary>
		/// Check if b is subset of a
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		static bool isSubset(const AABB& a, const AABB& b);

		static void expand(AABB& aabb, const real& factor = 0.0);

		static bool raycast(const AABB& aabb, const Vector2& start, const Vector2& direction);
		
	};


}
#endif