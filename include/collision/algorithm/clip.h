#ifndef PHYSICS2D_CLIP_H
#define PHYSICS2D_CLIP_H
#include "include/common/common.h"
#include "include/geometry/shape.h"
#include "include/math/linear/linear.h"
namespace Physics2D
{
	class Clipper
	{
	public:
		/// <summary>
		/// Clip shape B by shape A
		/// Only for polygon
		/// </summary>
		/// <param name="shapeA"></param>
		/// <param name="shapeB"></param>
		/// <param name="normal"></param>
		/// <returns></returns>
		static std::optional<std::vector<Vector2>> clip(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
		{
			return std::nullopt;
		}
	};
}
#endif
