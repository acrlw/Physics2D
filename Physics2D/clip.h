#ifndef PHYSICS2D_CLIP_H
#define PHYSICS2D_CLIP_H
#include "common.h"
#include "shape.h"
#include "linear.h"
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
		static Container::Vector<Vector2> dumpVertices(const ShapePrimitive& primitive);
		static ClipEdge findClipEdge(const Container::Vector<Vector2>& vertices, size_t index, const Vector2& normal);
		static ClipEdge dumpClipEdge(const ShapePrimitive& shape, const Container::Vector<Vector2>& vertices, const Vector2& normal);
		static std::pair<ClipEdge, ClipEdge> recognize(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& normal);
		static Container::Vector<PointPair> clip(const ClipEdge& clipEdgeA, const ClipEdge& clipEdgeB, const Vector2& normal);
	};
}
#endif
