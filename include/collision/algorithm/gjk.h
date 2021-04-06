#pragma once

#include "include/math/math.h"
#include "include/common/common.h"
#include "include/collision/contact.h"
#include "include/dynamics/shape.h"
#include "include/math/algorithm/graphics/2d.h"
namespace Physics2D
{

	struct Minkowski
	{
		Minkowski() = default;
		Minkowski(const Vector2& point_a, const Vector2& point_b) : pointA(point_a), pointB(point_b), result(pointA - pointB)
		{}
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
		Vector2 translation;
		number rotation = 0;
	};
	/// <summary>
	/// Simplex structure for gjk/epa test.
	/// By convention:
	/// 1 points: p0 , construct a single point
	/// 2 points: p0 -> p1, construct a segment
	/// 4 points: p0 -> p1 -> p2 -> p0, construct a triangle
	/// </summary>
	/// <returns></returns>
	struct Simplex
	{
		std::vector<Minkowski> vertices;
		bool isContainOrigin = false;
		bool containOrigin()
		{
			isContainOrigin = calculateContainOrigin(*this);
			return isContainOrigin;
		}
		static bool calculateContainOrigin(const Simplex& simplex)
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
	    void insert(const size_t& pos, const Minkowski& vertex)
		{
			vertices.insert(vertices.begin() + pos + 1, vertex);
		}
		bool contains(const Minkowski& minkowski)
		{
			return std::find(std::begin(vertices), std::end(vertices), minkowski) != std::end(vertices);
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
		/// <summary>
		/// Gilbert¨CJohnson¨CKeerthi distance algorithm
		/// </summary>
		/// <param name="shape_A"></param>
		/// <param name="shape_B"></param>
		/// <param name="iteration"></param>
		/// <returns>return initial simplex and whether collision exists</returns>
		static std::tuple<bool, Simplex> gjk(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const size_t& iteration = 50);
		/// <summary>
		/// Expanding Polygon Algorithm
		/// </summary>
		/// <param name="shape_A"></param>
		/// <param name="shape_B"></param>
		/// <param name="src">initial simplex</param>
		/// <param name="iteration">iteration times</param>
		/// <param name="epsilon">epsilon of iterated result</param>
		/// <returns>return expanded simplex</returns>
		static Simplex epa(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Simplex& src, const size_t& iteration = 50, const number& epsilon = 0.0001);
		/// <summary>
		/// Dump collision information from simplex
		/// </summary>
		/// <param name="shape_A"></param>
		/// <param name="shape_B"></param>
		/// <param name="simplex"></param>
		/// <returns></returns>
		static ContactInfo dumpInfo(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Simplex& simplex);
		
		static Minkowski support(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Vector2& direction);
		/// <summary>
		/// The edge closest to the origin
		/// size_t, size_t: the index of point of edge
		/// </summary>
		static std::tuple<size_t, size_t> findEdgeClosestToOrigin(const Simplex& simplex);
		
		static Vector2 findFarthestPoint(const ShapePrimitive& shape, const Vector2& direction);
		
		static std::optional<Minkowski> adjustSimplex(Simplex& simplex, const size_t& closest_1, const size_t& closest_2);
		
		static Vector2 calculateDirectionByEdge(const Vector2& p1, const Vector2& p2, bool pointToOrigin = true);
	};
}