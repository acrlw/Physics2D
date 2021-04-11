#ifndef PHYSICS2D_GJK_H
#define PHYSICS2D_GJK_H


#include "include/common/common.h"
#include "include/collision/contact.h"
#include "include/dynamics/shape.h"
#include "include/math/algorithm/graphics/2d.h"
namespace Physics2D
{

    struct Minkowski
    {
            Minkowski() = default;
            Minkowski(const Vector2& point_a, const Vector2& point_b);
            inline bool operator ==(const Minkowski& rhs)const;
            inline bool operator !=(const Minkowski& rhs)const;
            Vector2 pointA;
            Vector2 pointB;
            Vector2 result;
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
            bool containOrigin();
            static bool containOrigin(const Simplex& simplex);
    	
            void insert(const size_t& pos, const Minkowski& vertex);
            bool contains(const Minkowski& minkowski);
            Vector2 lastVertex()const;
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
            static Simplex epa(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Simplex& src, const size_t& iteration = 50, const real& epsilon = 0.0001);
            /// <summary>
            /// Dump collision information from simplex
            /// </summary>
            /// <param name="shape_A"></param>
            /// <param name="shape_B"></param>
            /// <param name="simplex"></param>
            /// <returns></returns>
            static ContactInfo dumpInfo(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Simplex& simplex);
            /// <summary>
            /// Support function.
            /// </summary>
            /// <param name="shape_A"></param>
            /// <param name="shape_B"></param>
            /// <param name="direction"></param>
            /// <returns></returns>
            static Minkowski support(const ShapePrimitive& shape_A, const ShapePrimitive& shape_B, const Vector2& direction);
            /// <summary>
            /// Find two points that can form an edge closest to origin of simplex
            /// </summary>
            /// <param name="simplex"></param>
            /// <returns></returns>
            static std::tuple<size_t, size_t> findEdgeClosestToOrigin(const Simplex& simplex);
            /// <summary>
            /// Find farthest projection point in given direction
            /// </summary>
            /// <param name="shape"></param>
            /// <param name="direction"></param>
            /// <returns></returns>
            static Vector2 findFarthestPoint(const ShapePrimitive& shape, const Vector2& direction);
            /// <summary>
            /// Adjust triangle simplex, remove the point that can not form a triangle that contains origin
            /// </summary>
            /// <param name="simplex"></param>
            /// <param name="closest_1"></param>
            /// <param name="closest_2"></param>
            /// <returns></returns>
            static std::optional<Minkowski> adjustSimplex(Simplex& simplex, const size_t& closest_1, const size_t& closest_2);
            /// <summary>
            /// Given two points, calculate the perpendicular vector and the orientation is user-defined.
            /// </summary>
            /// <param name="p1"></param>
            /// <param name="p2"></param>
            /// <param name="pointToOrigin"></param>
            /// <returns></returns>
            static Vector2 calculateDirectionByEdge(const Vector2& p1, const Vector2& p2, bool pointToOrigin = true);
    };
}


#endif
