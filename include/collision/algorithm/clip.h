#ifndef PHYSICS2D_CLIP_H
#define PHYSICS2D_CLIP_H
#include "include/common/common.h"
#include "include/geometry/shape.h"
#include "include/math/linear/linear.h"
#include "include/collision/algorithm/gjk.h"
namespace Physics2D
{
	struct ContactEdge
	{
		Vector2 point1;
		Vector2 point2;
	};
	class ContactGenerator
	{
	public:

		static std::optional<std::vector<PointPair>> clip(const ContactEdge& lhs, const ContactEdge& rhs, const bool& exchange = false);

		static std::optional<std::vector<PointPair>> generate(const ShapePrimitive& shape, const ContactEdge& edge, const Vector2& source, const PenetrationInfo& info, const bool& exchange = false);

	};

}
#endif
