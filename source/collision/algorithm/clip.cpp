#include "include/collision/algorithm/clip.h"
namespace Physics2D
{
	void ContactGenerator::recognize(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& normal)
	{
		auto typeA = shapeA.shape.get()->type();
		auto typeB = shapeB.shape.get()->type();
		if (typeA == Shape::Type::Point || typeA == Shape::Type::Circle || typeA == Shape::Type::Ellipse
			|| typeB == Shape::Type::Point || typeB == Shape::Type::Circle || typeB == Shape::Type::Ellipse)
			return;
		//normal: B -> A

	}

	std::vector<PointPair> ContactGenerator::clip(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
		const Vector2& normal)
	{
		std::vector<PointPair> result;

		return result;
	}
}
