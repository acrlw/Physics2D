#ifndef PHYSICS2D_CLIP_H
#define PHYSICS2D_CLIP_H
#include "include/common/common.h"
#include "include/geometry/shape.h"
#include "include/math/linear/linear.h"
#include "include/collision/algorithm/gjk.h"
namespace Physics2D
{
	
	class Clipper
	{
	public:
		/// <summary>
		/// Sutherland Hodgman Polygon Clipping
		///	All points is stored in counter clock winding.
		///	By convention:
		///		p0 -> p1 -> p2 -> p0 constructs a triangle
		/// </summary>
		/// <param name="polygon"></param>
		/// <param name="clipRegion"></param>
		/// <returns></returns>
		static std::vector<Vector2> sutherlandHodgmentPolygonClipping(const std::vector<Vector2>& polygon, const std::vector<Vector2>& clipRegion);
	};
	class ContactGenerator
	{
	public:
		struct ClipEdge
		{
			Vector2 p1;
			Vector2 p2;
			Vector2 normal;
		};
		static std::vector<Vector2> dumpVertices(const ShapePrimitive& primitive);
		static ClipEdge findClipEdge(const std::vector<Vector2>& vertices, size_t index, const Vector2& normal);
		static ClipEdge dumpClipEdge(const ShapePrimitive& shape, const std::vector<Vector2>& vertices, const Vector2& normal);
		static std::pair<ClipEdge, ClipEdge> recognize(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& normal);
		static std::vector<PointPair> clip(const ClipEdge& clipEdgeA, const ClipEdge& clipEdgeB, const Vector2& normal);
	};
	
}
#endif
