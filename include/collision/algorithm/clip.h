#ifndef PHYSICS2D_CLIP_H
#define PHYSICS2D_CLIP_H
#include "include/common/common.h"
#include "include/geometry/shape.h"
#include "include/math/linear/linear.h"
#include "include/collision/algorithm/gjk.h"
namespace Physics2D
{
	

	class ContactGenerator
	{
	public:
		static void recognize(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& normal);
		static std::vector<PointPair> clip(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& normal);
	};
	
}
#endif
