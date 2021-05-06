#include "include/collision/algorithm/mpr.h"

namespace Physics2D
{
	std::tuple<Vector2, Simplex> MPR::discover(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		Simplex simplex;
		Vector2 centerA = Matrix2x2(shapeA.rotation).multiply(shapeA.shape->center());
		Vector2 centerB = Matrix2x2(shapeB.rotation).multiply(shapeB.shape->center());
		Vector2 origin = shapeB.transform - shapeA.transform;
		Minkowski v0(centerA + shapeA.transform, centerB + shapeB.transform);
		Vector2 direction = centerB - centerA + origin;
		Minkowski v1 = GJK::support(shapeA, shapeB, direction);
		direction = GJK::calculateDirectionByEdge(v0.result, v1.result, true);
		Minkowski v2 = GJK::support(shapeA, shapeB, direction);
		simplex.vertices.emplace_back(v0);
		simplex.vertices.emplace_back(v1);
		simplex.vertices.emplace_back(v2);
		return std::make_tuple(centerB - centerA + origin, simplex);
	}

	std::tuple<bool, Simplex> MPR::refine(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                      const Simplex& source, const Vector2& centerToOrigin, const real& iteration)
	{
		Simplex simplex = source;
		bool isColliding = false;
		Vector2 v1, v2, direction;
		Minkowski lastRemoved;
		real counter = 0;
		while (counter++ < iteration)
		{
			v1 = simplex.vertices[1].result;
			v2 = simplex.vertices[2].result;
			direction = GJK::calculateDirectionByEdge(v1, v2, true);
			if (direction.dot(centerToOrigin) < 0)
			{
				direction.negate();
				isColliding = true;
			}
			Minkowski newVertex = GJK::support(shapeA, shapeB, direction);

			if (v1.fuzzyEqual(newVertex.result) || v2.fuzzyEqual(newVertex.result))
				break;

			lastRemoved = simplex.vertices[2];
			simplex.vertices[2] = newVertex;
		}
		return std::make_tuple(isColliding, simplex);
	}
}
