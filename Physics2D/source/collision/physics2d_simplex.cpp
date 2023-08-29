#include "physics2d_simplex.h"
namespace Physics2D
{
	bool SimplexVertexArray::containOrigin(bool strict)
	{
		isContainOrigin = containOrigin(*this, strict);
		return isContainOrigin;
	}

	bool SimplexVertexArray::containOrigin(const SimplexVertexArray& simplex, bool strict)
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

	void SimplexVertexArray::insert(const size_t& pos, const SimplexVertex& vertex)
	{
		vertices.insert(vertices.begin() + pos + 1, vertex);
	}

	bool SimplexVertexArray::contains(const SimplexVertex& vertex)
	{
		return std::find(std::begin(vertices), std::end(vertices), vertex) != std::end(vertices);
	}

	bool SimplexVertexArray::fuzzyContains(const SimplexVertex& vertex, const real& epsilon)
	{
		return std::find_if(std::begin(vertices), std::end(vertices),
			[=](const Physics2D::SimplexVertex& element)
			{
				return (vertex.result - element.result).lengthSquare() < epsilon;
			})
			!= std::end(vertices);
	}

	Vector2 SimplexVertexArray::lastVertex() const
	{
		if (vertices.size() == 2)
			return vertices[vertices.size() - 1].result;
		return vertices[vertices.size() - 2].result;
	}

	bool Simplex::containsOrigin(bool strict)
	{
		isContainOrigin = containOrigin(*this, strict);
		return isContainOrigin;
	}

	bool Simplex::containOrigin(const Simplex& simplex, bool strict)
	{
		switch (simplex.count)
		{
		case 1:
			return simplex.vertices[0].result.isOrigin() && !simplex.vertices[0].point[0].isOrigin() && !simplex.vertices[0].point[1].isOrigin();
		case 2:
			return GeometryAlgorithm2D::isPointOnSegment(simplex.vertices[0].result, simplex.vertices[1].result, { 0, 0 });
		case 3:
			return GeometryAlgorithm2D::triangleContainsOrigin(simplex.vertices[0].result, simplex.vertices[1].result, simplex.vertices[2].result);
		default:
			assert(false, "Simplex count is more than 3");
			return false;
		}
	}

	bool Simplex::contains(const SimplexVertex& vertex, const real& epsilon)
	{
		for (SimplexVertex& element : vertices)
			if (!element.isEmpty() && (element == vertex || element.result.fuzzyEqual(vertex.result, epsilon)))
				return true;
		return false;
	}
	void Simplex::addSimplexVertex(const SimplexVertex& vertex)
	{
		assert(count < 4);
		vertices[count] = vertex;
		++count;
	}

	void Simplex::removeByIndex(const Index& index)
	{
		assert(index < 3);
		if(index == 0)
		{
			vertices[0] = vertices[1];
			vertices[1] = vertices[2];
		}
		else if (index == 1)
			vertices[1] = vertices[2];
		
		vertices[2].clear();
		--count;
	}

	void Simplex::removeEnd()
	{
		vertices[2].clear();
		--count;
	}

	void Simplex::removeAll()
	{
		vertices[0].clear();
		vertices[1].clear();
		vertices[2].clear();
		count = 0;
	}
}
