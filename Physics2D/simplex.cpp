#include "simplex.h"
namespace Physics2D
{
	bool Simplex::containOrigin(bool strict)
	{
		isContainOrigin = containOrigin(*this, strict);
		return isContainOrigin;
	}

	bool Simplex::containOrigin(const Simplex& simplex, bool strict)
	{
		switch (simplex.vertices.size())
		{
		case 4:
		{
			return GeometryAlgorithm2D::triangleContainsOrigin(simplex.vertices[0].result,
				simplex.vertices[1].result, simplex.vertices[2].result);
		}
		case 2:
		{
			Vector2 oa = simplex.vertices[0].result * -1;
			Vector2 ob = simplex.vertices[1].result * -1;
			return GeometryAlgorithm2D::isPointOnSegment(oa, ob, { 0, 0 });

		}
		default:
			return false;
		}
	}

	void Simplex::insert(const size_t& pos, const Minkowski& vertex)
	{
		vertices.insert(vertices.begin() + pos + 1, vertex);
	}

	bool Simplex::contains(const Minkowski& minkowski)
	{
		return std::find(std::begin(vertices), std::end(vertices), minkowski) != std::end(vertices);
	}

	bool Simplex::fuzzyContains(const Minkowski& minkowski, const real& epsilon)
	{
		return std::find_if(std::begin(vertices), std::end(vertices),
			[=](const Minkowski& element)
			{
				return (minkowski.result - element.result).lengthSquare() < epsilon;
			})
			!= std::end(vertices);
	}

	Vector2 Simplex::lastVertex() const
	{
		if (vertices.size() == 2)
			return vertices[vertices.size() - 1].result;
		return vertices[vertices.size() - 2].result;
	}
}