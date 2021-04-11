#ifndef PHYSICS2D_BROADPHASE_DBVH_H
#define PHYSICS2D_BROADPHASE_DBVH_H
#include "include/collision/broadphase/aabb.h"

namespace Physics2D
{
	
	class DBVH
	{
		struct Node
		{
			
		};
		public:
			void insert(const ShapePrimitive& primitive);
			void insert(const AABB& aabb);
			void remove();
			void query(const Vector2& start, const Vector2& direction);
			void generatePairs();
			
		private:
	};
}
#endif