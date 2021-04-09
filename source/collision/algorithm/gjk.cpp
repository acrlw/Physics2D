#include "include/collision/algorithm/gjk.h"



namespace Physics2D
{
	
	bool Simplex::calculateContainOrigin(const Simplex& simplex)
	{
		switch (simplex.vertices.size())
		{
		case 4:
		{
			number a = 0, b = 0, c = 0;
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
			else
			{
				if (simplex.containOrigin())
				{
					found = true;
					break;
				}
				else
				{
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
				}
			}
			iter++;
		}

		return std::make_tuple(found, simplex);
	}

	Simplex GJK::epa(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Simplex& src,
	                 const size_t& iteration, const number& epsilon)
	{
		size_t iter = 0;
		Simplex edge;
		Simplex simplex = src;
		Vector2 normal;
		number originToEdge;
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

			const number d = p.result.dot(normal);
			const number diff = abs(d - originToEdge);
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
		number originToEdge = abs(normal.dot(simplex.vertices[index1].result));
		v_penetr = normal * originToEdge * -1;
		result.penetration = v_penetr;


		int dir = 1;
		if ((A_s1 - A_s2).lengthSquare() < (B_s1 - B_s2).lengthSquare())
		{
			witness = (A_s1 + A_s2) / 2;
		}
		else
		{
			witness = (B_s1 + B_s2) / 2;
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
		number min_dist = INT_MAX;

		size_t index1 = 0;
		size_t index2 = 0;

		for (size_t i = 0; i < simplex.vertices.size() - 1; i++)
		{
			Vector2 a = simplex.vertices[i].result;
			Vector2 b = simplex.vertices[i + 1].result;

			const Vector2 p = GeometryAlgorithm2D::pointToLineSegment(a, b, {0, 0});
			const number projection = p.length();


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
				const auto* polygon = dynamic_cast<const Polygon*>(shape.shape);
				Vector2 p0 = polygon->vertices()[0];
				number max = 0;
				target = polygon->vertices()[0];
				for (const Vector2& vertex : polygon->vertices())
				{
					number result = Vector2::dotProduct(vertex - p0, rot_dir);

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
				const auto* const circle = dynamic_cast<const Circle*>(shape.shape);
				target = direction.normal() * circle->radius();
				break;
			}
		case Shape::Type::Ellipse:
			{
				const auto* const ellipse = dynamic_cast<const Ellipse*>(shape.shape);
				const number a = ellipse->A();
				const number b = ellipse->B();
				if (rot_dir.x == 0.0f)
				{
					int sgn = direction.y < 0 ? -1 : 1;
					target.set(0, sgn * b);
				}
				else if (rot_dir.y == 0.0f)
				{
					int sgn = direction.x < 0 ? -1 : 1;
					target.set(sgn * a, 0);
				}
				else
				{
					float k = rot_dir.y / rot_dir.x;
					//line offset constant d
					float a2 = pow(a, 2);
					float b2 = pow(b, 2);
					float k2 = pow(k, 2);
					float d = sqrt((a2 + b2 * k2) / k2);
					float x1, y1;
					if (Vector2::dotProduct(Vector2(0, d), rot_dir) < 0)
						d = d * -1;
					x1 = k * d - (b2 * k2 * k * d) / (a2 + b2 * k2);
					y1 = (b2 * k2 * d) / (a2 + b2 * k2);
					target.set(x1, y1);
					//rotate the final vector
				}
				break;
			}
		case Shape::Type::Edge:
			{
				const auto* edge = dynamic_cast<const Edge*>(shape.shape);
				number dot1 = Vector2::dotProduct(edge->startPoint(), direction);
				number dot2 = Vector2::dotProduct(edge->endPoint(), direction);
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
				size_t index = -1;

				for (size_t i = 0; i < simplex.vertices.size() - 1; i++)
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
}
