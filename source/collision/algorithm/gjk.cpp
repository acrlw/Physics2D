#include "include/collision/algorithm/gjk.h"


namespace Physics2D
{
	bool Simplex::containOrigin()
	{
		isContainOrigin = containOrigin(*this);
		return isContainOrigin;
	}

	bool Simplex::containOrigin(const Simplex& simplex)
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
				Vector2 oa = simplex.vertices[0].result * -1;
				Vector2 ob = simplex.vertices[1].result * -1;
				return Vector2::crossProduct(oa, ob) == 0;
			}
		case 1:
			{
				return simplex.vertices[0].result.length() == 0;
				break;
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

	Vector2 Simplex::lastVertex() const
	{
		if (vertices.size() == 2)
			return vertices[vertices.size() - 1].result;
		return vertices[vertices.size() - 2].result;
	}

	std::tuple<bool, Simplex> GJK::gjk(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B,
	                                   const size_t& iteration)
	{
		Simplex simplex;
		bool found = false;
		Vector2 direction = shape_B.transform - shape_A.transform;
		Minkowski diff = support(shape_A, shape_B, direction);
		simplex.vertices.emplace_back(diff);
		direction.negate();
		size_t iter = 0;
		std::vector<Minkowski> removed;
		while (iter <= iteration)
		{
			diff = support(shape_A, shape_B, direction);
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

	Simplex GJK::epa(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Simplex& src,
	                 const size_t& iteration, const real& epsilon)
	{
		size_t iter = 0;
		Simplex edge;
		Simplex simplex = src;
		Vector2 normal;
		real originToEdge;
		Minkowski p;
		while (iter <= iteration)
		{
			auto [index1, index2] = findEdgeClosestToOrigin(simplex);
			normal = calculateDirectionByEdge(simplex.vertices[index1].result, simplex.vertices[index2].result, false).
				normal();
			originToEdge = abs(normal.dot(simplex.vertices[index1].result));
			//new minkowski point
			p = support(shape_A, shape_B, normal);

			if (simplex.contains(p))
				break;

			const real d = p.result.dot(normal);
			const real diff = abs(d - originToEdge);
			//if distance of origin to edge is close enough to the distance of origin to edge point
			if (diff < epsilon)
				break;

			simplex.insert(index1, p);
			iter++;
		}
		return simplex;
	}

	ContactInfo GJK::dumpInfo(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Simplex& simplex)
	{
		ContactInfo result;
		result.isCollide = simplex.isContainOrigin;
		auto [index1, index2] = findEdgeClosestToOrigin(simplex);
		const Vector2 A_s1 = simplex.vertices[index1].pointA;
		const Vector2 A_s2 = simplex.vertices[index2].pointA;
		const Vector2 B_s1 = simplex.vertices[index1].pointB;
		const Vector2 B_s2 = simplex.vertices[index2].pointB;

		Vector2 normal, v_penetr;
		Vector2 witness, mirror;
		normal = calculateDirectionByEdge(simplex.vertices[index1].result, simplex.vertices[index2].result, false).
			normal();
		real originToEdge = abs(normal.dot(simplex.vertices[index1].result));
		v_penetr = normal * originToEdge * -1;
		result.penetration = v_penetr;


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
		mirror = witness + v_penetr * dir;
		if (dir < 0)
		{
			Vector2 temp = witness;
			witness = mirror;
			mirror = temp;
		}
		result.contactA = witness;
		result.contactB = mirror;
		return result;
	}

	Minkowski GJK::support(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Vector2& direction)
	{
		Vector2 p1 = findFarthestPoint(shape_A, direction);
		Vector2 p2 = findFarthestPoint(shape_B, direction * -1);
		return Minkowski(p1, p2);
	}

	std::tuple<size_t, size_t> GJK::findEdgeClosestToOrigin(const Simplex& simplex)
	{
		real min_dist = INT_MAX;

		size_t index1 = 0;
		size_t index2 = 0;

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
				const Polygon* polygon = dynamic_cast<const Polygon*>(shape.shape);
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
				const Circle* circle = dynamic_cast<const Circle*>(shape.shape);
				target = direction.normal() * circle->radius();
				break;
			}
		case Shape::Type::Ellipse:
			{
				const Ellipse* ellipse = dynamic_cast<const Ellipse*>(shape.shape);
				target = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), rot_dir);
				break;
			}
		case Shape::Type::Edge:
			{
				const Edge* edge = dynamic_cast<const Edge*>(shape.shape);
				real dot1 = Vector2::dotProduct(edge->startPoint(), direction);
				real dot2 = Vector2::dotProduct(edge->endPoint(), direction);
				target = dot1 > dot2 ? edge->startPoint() : edge->endPoint();
				break;
			}
		case Shape::Type::Point:
			{
				return dynamic_cast<const Point*>(shape.shape)->position();
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
