#include "include/collision/algorithm/gjk.h"


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
				real a = 0, b = 0, c = 0;
				Vector2 oa = simplex.vertices[0].result * -1;
				Vector2 ob = simplex.vertices[1].result * -1;
				Vector2 oc = simplex.vertices[2].result * -1;

				a = Vector2::crossProduct(oa, ob);
				b = Vector2::crossProduct(ob, oc);
				c = Vector2::crossProduct(oc, oa);

				if ((a <= 0 && b <= 0 && c <= 0) ||
					(a >= 0 && b >= 0 && c >= 0))
					return true;
				return false;
			}
		case 2:
			{
				if(strict)
				{
					Vector2 oa = simplex.vertices[0].result * -1;
					Vector2 ob = simplex.vertices[1].result * -1;
					return GeometryAlgorithm2D::isPointOnSegment(oa, ob, { 0, 0 });
				}
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
		auto result = std::find_if(std::begin(vertices), std::end(vertices),
		                           [=](const Minkowski& element)
		                           {
			                           return (minkowski.result - element.result).lengthSquare() < epsilon;
		                           });
		return result != std::end(vertices);
	}

	Vector2 Simplex::lastVertex() const
	{
		if (vertices.size() == 2)
			return vertices[vertices.size() - 1].result;
		return vertices[vertices.size() - 2].result;
	}

	std::tuple<bool, Simplex> GJK::gjk(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                   const size_t& iteration)
	{
		Simplex simplex;
		bool found = false;
		Vector2 direction = shapeB.transform - shapeA.transform;
		
		if (direction.fuzzyEqual({ 0, 0 }))
			direction.set(1, 1);

		Minkowski diff = support(shapeA, shapeB, direction);
		simplex.vertices.emplace_back(diff);
		direction.negate();
		size_t iter = 0;
		std::vector<Minkowski> removed;
		while (iter <= iteration)
		{
			diff = support(shapeA, shapeB, direction);
			simplex.vertices.emplace_back(diff);
			if (simplex.vertices.size() == 3)
				simplex.vertices.emplace_back(simplex.vertices[0]);

			if (simplex.lastVertex().dot(direction) <= 0)
				break;
			if (simplex.containOrigin())
			{
				found = true;
				break;
			}
			//if not contain origin
			//find edge closest to origin
			//reconstruct simplex
			//find the point that is not belong to the edge closest to origin
			//if found, there is no more minkowski difference, exit loop
			//if not, add the point to the list

			auto [index1, index2] = findEdgeClosestToOrigin(simplex);
			direction = calculateDirectionByEdge(simplex.vertices[index1].result,
			                                     simplex.vertices[index2].result, true);

			auto result = adjustSimplex(simplex, index1, index2);
			if (result.has_value())
			{
				if (std::find(std::begin(removed), std::end(removed), result.value()) != removed.end())
					break;

				removed.emplace_back(result.value());
			}
			iter++;
		}

		return std::make_tuple(found, simplex);
	}

	Simplex GJK::epa(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Simplex& src,
	                 const size_t& iteration, const real& epsilon)
	{
		size_t iter = 0;
		Simplex edge;
		Simplex simplex = src;
		Vector2 normal;
		Minkowski p;
		while (iter <= iteration)
		{
			auto [index1, index2] = findEdgeClosestToOrigin(simplex);
			
			normal = calculateDirectionByEdge(simplex.vertices[index1].result, simplex.vertices[index2].result, false).
				normal();

			if (GeometryAlgorithm2D::isPointOnSegment(simplex.vertices[index1].result, simplex.vertices[index2].result, { 0, 0 }))
				normal.negate();
			
			//new minkowski point
			p = support(shapeA, shapeB, normal);

			if (simplex.contains(p))
				break;

			if (simplex.fuzzyContains(p, epsilon))
				break;

			simplex.insert(index1, p);
			iter++;
		}
		return simplex;
	}

	PenetrationInfo GJK::dumpInfo(const PenetrationSource& source)
	{
		PenetrationInfo result;
		Vector2 edge1 = source.a1 - source.b1;
		Vector2 edge2 = source.a2 - source.b2;
		Vector2 normal = calculateDirectionByEdge(edge1, edge2, false).
			normal();
		real originToEdge = abs(normal.dot(edge1));
		result.normal = normal * -1;
		result.penetration = originToEdge;
		return result;
	}

	Minkowski GJK::support(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& direction)
	{
		Vector2 p1 = findFarthestPoint(shapeA, direction);
		Vector2 p2 = findFarthestPoint(shapeB, direction * -1);
		return Minkowski(p1, p2);
	}

	std::tuple<size_t, size_t> GJK::findEdgeClosestToOrigin(const Simplex& simplex)
	{
		real min_dist = INT_MAX;

		size_t index1 = 0;
		size_t index2 = 0;

		if (simplex.vertices.size() == 2)
			return std::make_tuple(0, 1);

		for (size_t i = 0; i < simplex.vertices.size() - 1; i++)
		{
			Vector2 a = simplex.vertices[i].result;
			Vector2 b = simplex.vertices[i + 1].result;

			const Vector2 p = GeometryAlgorithm2D::pointToLineSegment(a, b, {0, 0});
			const real projection = p.length();


			if (min_dist > projection)
			{
				index1 = i;
				index2 = i + 1;
				min_dist = projection;
			}
			else if (realEqual(min_dist, projection))
			{
				real length1 = a.lengthSquare() + b.lengthSquare();
				real length2 = simplex.vertices[index1].result.lengthSquare() + simplex.vertices[index2].result.
					lengthSquare();
				if (length1 < length2)
				{
					index1 = i;
					index2 = i + 1;
				}
			}
		}
		return std::make_tuple(index1, index2);
	}

	Vector2 GJK::findFarthestPoint(const ShapePrimitive& shape, const Vector2& direction)
	{
		Vector2 target;
		Matrix2x2 rot(-shape.rotation);
		Vector2 rot_dir = rot.multiply(direction);
		switch (shape.shape->type())
		{
		case Shape::Type::Polygon:
			{
				const Polygon* polygon = dynamic_cast<const Polygon*>(shape.shape.get());
				Vector2 p0 = polygon->vertices()[0];
				real max = 0;
				target = polygon->vertices()[0];
				for (const Vector2& vertex : polygon->vertices())
				{
					real result = Vector2::dotProduct(vertex - p0, rot_dir);

					if (max < result)
					{
						max = result;
						target = vertex;
					}
				}
				break;
			}
		case Shape::Type::Circle:
			{
				const Circle* circle = dynamic_cast<const Circle*>(shape.shape.get());
				target = direction.normal() * circle->radius() + shape.transform;
				return target;
			}
		case Shape::Type::Ellipse:
			{
				const Ellipse* ellipse = dynamic_cast<const Ellipse*>(shape.shape.get());
				target = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), rot_dir);
				break;
			}
		case Shape::Type::Edge:
			{
				const Edge* edge = dynamic_cast<const Edge*>(shape.shape.get());
				real dot1 = Vector2::dotProduct(edge->startPoint(), direction);
				real dot2 = Vector2::dotProduct(edge->endPoint(), direction);
				target = dot1 > dot2 ? edge->startPoint() : edge->endPoint();
				break;
			}
		case Shape::Type::Point:
			{
				return dynamic_cast<const Point*>(shape.shape.get())->position();
			}
		default:
			break;
		}
		rot.setAngle(shape.rotation);
		target = rot.multiply(target);
		target += shape.transform;
		return target;
	}

	std::optional<Minkowski> GJK::adjustSimplex(Simplex& simplex, const size_t& closest_1, const size_t& closest_2)
	{
		switch (simplex.vertices.size())
		{
		case 4: //only adjust for triangle from gjk
			{
				int32_t index = -1;

				for (int32_t i = 0; i < simplex.vertices.size() - 1; i++)
					if (i != closest_1 && i != closest_2)
						index = i;

				Minkowski target = simplex.vertices[index];

				simplex.vertices.erase(simplex.vertices.begin() + index);
				simplex.vertices.erase(simplex.vertices.begin() + simplex.vertices.size() - 1);
				return std::optional<Minkowski>(target);
			}
		default:
			return std::nullopt;
		}
	}

	Vector2 GJK::calculateDirectionByEdge(const Vector2& p1, const Vector2& p2, bool pointToOrigin)
	{
		const Vector2 ao = p1 * -1;
		const Vector2 ab = p2 - p1;
		Vector2 perpendicularOfAB = ab.perpendicular();
		if ((Vector2::dotProduct(ao, perpendicularOfAB) < 0 && pointToOrigin) || (
			Vector2::dotProduct(ao, perpendicularOfAB) > 0 && !pointToOrigin))
			perpendicularOfAB.negate();
		return perpendicularOfAB;
	}

	PointPair GJK::distance(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const real& iteration,
	                           const real& epsilon)
	{
		PointPair result;
		Simplex simplex;
		Vector2 direction = shapeB.transform - shapeA.transform;
		Minkowski m = support(shapeA, shapeB, direction);
		simplex.vertices.emplace_back(m);
		direction.negate();
		m = support(shapeA, shapeB, direction);
		simplex.vertices.emplace_back(m);
		int iter = 0;
		while (iter++ < iteration)
		{
			direction = calculateDirectionByEdge(simplex.vertices[0].result, simplex.vertices[1].result, true);
			m = support(shapeA, shapeB, direction);

			if (simplex.contains(m))
				break;

			//for ellipse
			if (simplex.fuzzyContains(m, epsilon))
				break;

			simplex.vertices.emplace_back(m);
			simplex.vertices.emplace_back(simplex.vertices[0]);
			auto [index1, index2] = findEdgeClosestToOrigin(simplex);
			adjustSimplex(simplex, index1, index2);
		}
		
		auto source = dumpSource(simplex);
		
		const Vector2 A_s1 = source.a1;
		const Vector2 A_s2 = source.a2;
		const Vector2 B_s1 = source.b1;
		const Vector2 B_s2 = source.b2;

		const real diffEpsilonA = (A_s1 - A_s2).lengthSquare();
		const real diffEpsilonB = (B_s1 - B_s2).lengthSquare();
		//two point
		if (diffEpsilonA < epsilon && diffEpsilonB < epsilon)
		{
			result.pointA = (A_s1 + A_s2) * 0.5;
			result.pointB = (B_s1 + B_s2) * 0.5;
		}
		//point a and edge b
		if (diffEpsilonA < epsilon && diffEpsilonB > epsilon)
		{
			result.pointA = (A_s1 + A_s2) * 0.5;
			result.pointB = GeometryAlgorithm2D::pointToLineSegment(B_s1, B_s2, result.pointA);
		}
		//point b and edge a
		if (diffEpsilonA > epsilon && diffEpsilonB < epsilon)
		{
			result.pointB = (B_s1 + B_s2) * 0.5;
			result.pointA = GeometryAlgorithm2D::pointToLineSegment(A_s1, A_s2, result.pointB);
		}
		return result;
	}

	PenetrationSource GJK::dumpSource(const Simplex& simplex)
	{
		PenetrationSource result;
		auto [index1, index2] = findEdgeClosestToOrigin(simplex);
		result.a1 = simplex.vertices[index1].pointA;
		result.a2 = simplex.vertices[index2].pointA;
		result.b1 = simplex.vertices[index1].pointB;
		result.b2 = simplex.vertices[index2].pointB;
		return result;
	}
	
	PointPair GJK::dumpContacts(const PenetrationSource& source, const PenetrationInfo& info)
	{
		PointPair result;
		const Vector2 A_s1 = source.a1;
		const Vector2 A_s2 = source.a2;
		const Vector2 B_s1 = source.b1;
		const Vector2 B_s2 = source.b2;
		
		Vector2 witness;

		int dir = 1;
		if ((A_s1 - A_s2).lengthSquare() < (B_s1 - B_s2).lengthSquare())
		{
			witness = (A_s1 + A_s2) * (0.5f);
		}
		else
		{
			witness = (B_s1 + B_s2) * (0.5f);
			dir = dir * -1;
		}
		Vector2 mirror = witness + info.normal * info.penetration * dir;
		if (dir < 0)
		{
			const Vector2 temp = witness;
			witness = mirror;
			mirror = temp;
		}
		result.pointA = witness;
		result.pointB = mirror;
		return result;
	}
	Minkowski::Minkowski(const Vector2& point_a, const Vector2& point_b) : pointA(point_a), pointB(point_b),
	                                                                       result(pointA - pointB)
	{
	}

	bool Minkowski::operator ==(const Minkowski& rhs) const
	{
		return pointA == rhs.pointA && pointB == rhs.pointB;
	}

	bool Minkowski::operator !=(const Minkowski& rhs) const
	{
		return !(pointA == rhs.pointA && pointB == rhs.pointB);
	}
}
