#ifndef PHYSICS2D_CLIP_H
#define PHYSICS2D_CLIP_H
#include "../../common/common.h"
#include "../../geometry/shape.h"
#include "../../math/linear/linear.h"
#include "gjk.h"
namespace Physics2D
{
	class ContactGenerator
	{
	public:
		struct ClipEdge
		{
			Vector2 p1;
			Vector2 p2;
			Vector2 normal;
			bool isEmpty()const
			{
				return p1.isOrigin() && p2.isOrigin();
			}
		};
		static std::vector<Vector2> dumpVertices(const ShapePrimitive& primitive);
		static ClipEdge findClipEdge(const std::vector<Vector2>& vertices, size_t index, const Vector2& normal);
		static ClipEdge dumpClipEdge(const ShapePrimitive& shape, const std::vector<Vector2>& vertices, const Vector2& normal);
		static std::pair<ClipEdge, ClipEdge> recognize(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& normal);
		static std::vector<PointPair> clip(const ClipEdge& clipEdgeA, const ClipEdge& clipEdgeB, const Vector2& normal);
	};
}
#endif
