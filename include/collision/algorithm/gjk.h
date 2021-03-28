#pragma once

#include "include/math/math.h"
#include "include/common/common.h"
#include "include/collision/contact.h"
#include "include/dynamics/shape.h"
namespace Physics2D
{

	struct Minkowski
	{
		Minkowski() = default;
		Minkowski(const Vector2& _pointA, const Vector2& _pointB) : pointA(_pointA), pointB(_pointB), result(pointA - pointB)
		{

		}
		inline bool operator ==(const Minkowski& rhs)const
		{
			return pointA == rhs.pointA && pointB == rhs.pointB;
		}
		inline bool operator !=(const Minkowski& rhs)const
		{
			return !(pointA == rhs.pointA && pointB == rhs.pointB);
		}
		Vector2 pointA;
		Vector2 pointB;
		Vector2 result;
	};
	/// <summary>
	/// Shape Collision Test Primitive.
	/// Including shape type, position, angle
	/// </summary>
	struct ShapePrimitive
	{
		Shape * shape = nullptr;
		Vector2 position;
		number angle = 0;
	};
	/// <summary>
	/// Simplex structure for gjk/epa test.
	/// By convention, there are at least two points in simplex.
	/// 2 points: p0 -> p0 , construct a single point
	/// 3 points: p0 -> p1 -> p1, construct a segment
	/// 4 points: p0 -> p1 -> p2 -> p0, construct a triangle
	/// </summary>
	/// <returns></returns>
	struct Simplex
	{
		std::vector<Minkowski> vertices;
		inline bool containOrigin()
		{
			return isContainOrigin(*this);
		}
		static inline bool isContainOrigin(const Simplex& simplex)
		{
			switch (simplex.vertices.size())
			{
            case 4:
	            {
					number a = 0, b = 0, c = 0;
					Vector2 origin;
					Vector2 oa = origin - simplex.vertices[0].result;
					Vector2 ob = origin - simplex.vertices[1].result;
					Vector2 oc = origin - simplex.vertices[2].result;

					a = Vector2::crossProduct(a, ob);
					b = Vector2::crossProduct(ob, oc);
					c = Vector2::crossProduct(oc, oa);

					if ((a <= 0 && b <= 0 && c <= 0) ||
						(a >= 0 && b >= 0 && c >= 0))
						return true;
					return false;
	            }
            case 2:
	            {
					Vector2 origin;
					Vector2 oa = origin - simplex.vertices[0].result;
					Vector2 ob = origin - simplex.vertices[1].result;
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
		inline bool insert(const Simplex& edge, const Minkowski& vertex)
		{
			size_t targetIndex = -1;
			for (size_t i = 0; i < vertices.size() - 1; i++)
				if (vertices[i].result == edge.vertices[0].result)
					targetIndex = i;
				
			vertices.insert(vertices.begin() + targetIndex + 1, vertex);
			return targetIndex == -1;
		}
		Vector2 lastVertex()const
		{
			if (vertices.size() == 2)
				return vertices[vertices.size() - 1].result;
			else 
				return vertices[vertices.size() - 2].result;
		}
	};



	class GJK
	{
	public:
		static ContactInfo test(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B)
		{

		}
		static std::tuple<bool, Simplex> gjk(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B)
		{
			Simplex simplex;
			bool found = false;
			Vector2 direction = shape_B.position - shape_A.position;
			Minkowski diff = support(shape_A, shape_B, direction);
			simplex.vertices.emplace_back(diff);
			direction.negate();
			int iteration = 0;
			std::vector<Minkowski> removed;
			while(iteration <= GJKIteration)
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
						direction = calculateDirectionByEdge(simplex.vertices[index1].result, simplex.vertices[index2].result, true);
						
						auto result = adjustSimplex(simplex, index1, index2);
						if(result.has_value())
						{
							if (std::find(std::begin(removed), std::end(removed), result.value()) != removed.end())
								break;
							
							removed.emplace_back(result.value());
						}
					}
				}
				iteration++;
			}

			return std::make_tuple(found, simplex);
		}
		static ContactInfo epa(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Simplex& src)
		{
			ContactInfo result;
			result.isCollide = true;
			size_t iteration = 0;
			Simplex edge;
			Simplex simplex = src;
			Vector2 normal, witness, mirror;
			number originToEdge;
			Minkowski p;
			Vector2 A_s1, A_s2, B_s1, B_s2;
			while(iteration <= GJKIteration)
			{
				//edge = findClosestEdge(simplex);
				//normal = calculateDirectionByEdge(edge, false).normal();
				originToEdge = abs(normal.dot(edge.vertices[0].result));
				p = support(shape_A, shape_B, normal);

				number d = p.result.dot(normal);
				number diff = abs(d - originToEdge);
				
				bool isExisted = false;
				for (const Minkowski& vertex : simplex.vertices)
				{
					if(vertex == p)
					{
						isExisted = true;
						break;
					}
				}
				if (isExisted || diff < EPAEPSILON)
				{
					result.penetrationVector = normal * originToEdge * -1;
					A_s1 = edge.vertices[0].pointA;
					A_s2 = edge.vertices[1].pointA;
					B_s1 = edge.vertices[0].pointB;
					B_s2 = edge.vertices[1].pointB;
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
					A_s1 += shape_A.position;
					A_s2 += shape_A.position;
					B_s1 += shape_B.position;
					B_s2 += shape_B.position;
					witness += shape_A.position;
					mirror = witness + result.penetrationVector * dir;
					if (dir < 0)
					{
						Vector2 temp = witness;
						witness = mirror;
						mirror = temp;
					}
					result.contactA = mirror;
					result.contactB = witness;
				}
				else
					simplex.insert(edge, p);
				iteration++;
			}
			return result;
		}
		static Minkowski support(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Vector2& direction)
		{
			
			Vector2 p1 = findFarthestPoint(shape_A, direction);
			Vector2 p2 = findFarthestPoint(shape_B, direction * -1);
			return Minkowski(p1, p2);
		}
		/// <summary>
		/// The edge closest to the origin
		/// bool: find or not
		/// size_t, size_t: the index of point of edge
		/// </summary>
		static std::tuple<size_t, size_t> findEdgeClosestToOrigin(const Simplex& simplex)
		{
			int min_dist = INT_MAX;
			
			size_t index1 = 0;
			size_t index2 = 0;
			
			for(size_t i = 0;i < simplex.vertices.size() - 1;i++)
			{
				size_t j = i + 1;
				Vector2 a = simplex.vertices[i].result;
				Vector2 b = simplex.vertices[j].result;
				Vector2 ab = b - a;
				Vector2 ao = a * -1;
				Vector2 perpendicularOfAB = ab.perpendicular();
				Vector2 e = perpendicularOfAB;
				if (ao.dot(perpendicularOfAB) < 0)
					e.negate();
				number projection = ao.dot(e.normal());
				if(min_dist > projection)
				{
					index1 = i;
					index2 = j;
					min_dist = projection;
				}
			}
			return std::make_tuple(index1, index2);
		}
		static Vector2 findFarthestPoint(const ShapePrimitive& shape, const Vector2& direction)
		{
			Vector2 target;
			Rotation2 rot = Rotation2(-1 * shape.angle);
			Vector2 rot_dir = rot.multiply(direction);
			switch (shape.shape->type())
			{
			case Shape::Type::Polygon:
			{
				auto polygon = dynamic_cast<const Polygon*>(shape.shape);
				Vector2 p0 = polygon->vertices()[0];
				number max = 0;
				target = polygon->vertices()[0];
				for (const Vector2& vertex : polygon->vertices())
				{
					number result = Vector2::dotProduct(vertex - p0, rot_dir);

					if(max < result)
					{
						max = result;
						target = vertex;
					}
				}
				break;
			}
			case Shape::Type::Circle:
			{
				auto circle = dynamic_cast<const Circle*>(shape.shape);
				target = rot_dir.normalize() * circle->radius();
				break;
			}
			case Shape::Type::Ellipse:
			{
				auto ellipse = dynamic_cast<const Ellipse*>(shape.shape);
				number a = ellipse->A();
				number b = ellipse->A();
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
					rot.setAngle(shape.angle);
				}
				break;
			}
			case Shape::Type::Edge:
			{
				auto edge = dynamic_cast<const Edge*>(shape.shape);
				number dot1, dot2;
				dot1 = Vector2::dotProduct(edge->startPoint(), direction);
				dot2 = Vector2::dotProduct(edge->endPoint(), direction);
				target = dot1 > dot2 ? edge->startPoint() : edge->endPoint();
				break;
			}
			default:
				break;
			}
			return target;
		}
		static std::optional<Minkowski> adjustSimplex(Simplex& simplex, const size_t& closest_1, const size_t& closest_2)
		{
			switch (simplex.vertices.size())
			{
				case 4: //triangle
				{
					size_t index = -1;
						
					for(size_t i = 0;i < simplex.vertices.size() - 1;i++)
						if(i != closest_1 && i != closest_2)
							index = i;
						
					Minkowski target = simplex.vertices[index];

					simplex.vertices.erase(simplex.vertices.begin() + index);
					simplex.vertices.erase(simplex.vertices.begin() + simplex.vertices.size() - 1);
					return target;
				}
				default:
					return std::nullopt;
			}
		}
		static Vector2 calculateDirectionByEdge(const Vector2& p1, const Vector2& p2, bool pointToOrigin = true)
		{
			Vector2 ao = p1 * -1;
			Vector2 ab = p2 - p1;
			Vector2 perpendicularOfAB = ab.perpendicular();
			if ((Vector2::dotProduct(ao,perpendicularOfAB) < 0 && pointToOrigin) || (Vector2::dotProduct(ao, perpendicularOfAB) > 0 && !pointToOrigin))
				perpendicularOfAB.negate();
			return perpendicularOfAB;
		}
	};
}