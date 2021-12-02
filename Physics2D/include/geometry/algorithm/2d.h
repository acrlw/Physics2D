#ifndef PHYSICS2D_ALGORITHM_GRAPHICS_2D
#define PHYSICS2D_ALGORITHM_GRAPHICS_2D

#include "../../math/linear/linear.h"
#include "../../common/common.h"
namespace Physics2D
{
	namespace GeometryAlgorithm2D
	{
		class Clipper
		{
		public:
			/// <summary>
			/// Sutherland Hodgman Polygon Clipping
			///	All points is stored in counter clock winding.
			///	By convention:
			///		p0 -> p1 -> p2 -> p0 constructs a triangle
			/// </summary>
			/// <param name="polygon"></param>
			/// <param name="clipRegion"></param>
			/// <returns></returns>
			static std::vector<Vector2> sutherlandHodgmentPolygonClipping(const std::vector<Vector2>& polygon, const std::vector<Vector2>& clipRegion);
		};
		/// <summary>
		/// Check if point a,b,c are collinear using triangle area method
		/// </summary>
		/// <param name="a">point a</param>
		/// <param name="b">point b</param>
		/// <param name="c">point c</param>
		/// <returns></returns>
		bool isCollinear(const Vector2& a, const Vector2& b, const Vector2& c);
		/// <summary>
		/// Check if point c is on line segment ab using line projection and set-union method
		/// </summary>
		/// <param name="a">end of segment a</param>
		/// <param name="b">end of segment b</param>
		/// <param name="c">point c</param>
		/// <returns></returns>
		bool isPointOnSegment(const Vector2& a, const Vector2& b, const Vector2& c);
		/// <summary>
		/// Check if point c is on line segment ab, given a,b,c is already collinear by calculating cross product
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		bool fuzzyIsPointOnSegment(const Vector2& a, const Vector2& b, const Vector2& c, const real& epsilon = Constant::GeometryEpsilon);
		bool fuzzyIsCollinear(const Vector2& a, const Vector2& b, const Vector2& c);
		/// <summary>
		/// Calculate intersected point between line ab and line cd.
		/// Return if there is a actual intersected point.
		/// Notices: overlapping is NOT considered as a kind of intersection situation in this function
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <param name="d"></param>
		/// <returns></returns>
		std::optional<Vector2> lineSegmentIntersection(const Vector2& a, const Vector2& b, const Vector2& c, const Vector2& d);
		/// <summary>
		/// line intersection
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <param name="d"></param>
		/// <returns></returns>
		Vector2 lineIntersection(const Vector2& p1, const Vector2& p2, const Vector2& q1, const Vector2& q2);
		/// <summary>
		/// Calculate the center of circum-circle from triangle abc
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		std::optional<Vector2> triangleCircumcenter(const Vector2& a, const Vector2& b, const Vector2& c);
		/// <summary>
		/// Calculate the center of inscribed-circle from triangle abc
		/// If a,b,c can not form a triangle, return nothing
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		std::optional<Vector2> triangleIncenter(const Vector2& a, const Vector2& b, const Vector2& c);
		/// <summary>
		/// Calculate circum-circle given three points that can form a triangle
		/// If a,b,c can not form a triangle, return nothing
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		std::optional<std::tuple<Vector2, real>> calculateCircumcircle(const Vector2& a, const Vector2& b, const Vector2& c);
		/// <summary>
		/// Calculate inscribed circle given three points that can form a triangle
		/// If a,b,c can not form a triangle, return nothing
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		std::optional<std::tuple<Vector2, real>> calculateInscribedCircle(const Vector2& a, const Vector2& b, const Vector2& c);
		/// <summary>
		/// Check if a polygon is convex
		/// </summary>
		/// <param name="vertices"></param>
		/// <returns></returns>
		bool isConvexPolygon(const std::vector<Vector2>& vertices);
		/// <summary>
		/// Convex hull algorithm: Graham Scan. Given a series of points, find the convex polygon that can contains all of these points.
		/// </summary>
		/// <param name="vertices"></param>
		/// <returns></returns>
		std::vector<Vector2> grahamScan(const std::vector<Vector2>& vertices);
		/// <summary>
		/// calculate point on line segment ab that is the shortest length to point p
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		Vector2 pointToLineSegment(const Vector2& a, const Vector2& b, const Vector2& p);
		/// <summary>
		/// Calculate point on ellipse that is the shortest length to point p(aka projection point)
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="p"></param>
		/// <returns></returns>
		Vector2 shortestLengthPointOfEllipse(const real& a, const real& b, const Vector2& p, const real& epsilon = 0.00000001f);
		/// <summary>
		/// Calculate the centroid of triangle.
		/// </summary>
		/// <param name="a1"></param>
		/// <param name="a2"></param>
		/// <param name="a3"></param>
		/// <returns></returns>
		Vector2 triangleCentroid(const Vector2& a1, const Vector2& a2, const Vector2& a3);
		/// <summary>
		/// Calculate the area of triangle use cross product
		/// </summary>
		/// <param name="a1"></param>
		/// <param name="a2"></param>
		/// <param name="a3"></param>
		/// <returns></returns>
		real triangleArea(const Vector2& a1, const Vector2& a2, const Vector2& a3);
		/// <summary>
		/// Calculate mass center of 'convex' polygon
		/// </summary>
		/// <param name="vertices"></param>
		/// <returns></returns>
		Vector2 calculateCenter(const std::vector<Vector2>& vertices);
		/// <summary>
		/// Calculate two points on line segment and ellipse respectively. The length of two points is the shortest distance of line segment and ellipse
		/// </summary>
		/// <param name="a">major axis a</param>
		/// <param name="b">minor axis b</param>
		/// <param name="p1">line segment point 1</param>
		/// <param name="p2">line segment point 2</param>
		/// <returns></returns>
		std::tuple<Vector2, Vector2> shortestLengthLineSegmentEllipse(const real& a, const real& b, const Vector2& p1, const Vector2& p2);
		/// <summary>
		/// Calculate point on line segment ab, if point 'p' can cast ray in 'dir' direction on line segment ab.
		/// Algorithm from wikipedia 'Line-line intersection'
		/// </summary>
		/// <param name="p">ray start point</param>
		/// <param name="dir">ray direction</param>
		/// <param name="a">line segment point a</param>
		/// <param name="b">line segment point b</param>
		/// <returns></returns>
		std::optional<Vector2> raycast(const Vector2& p, const Vector2& dir, const Vector2& a, const Vector2& b);
		std::optional<std::pair<Vector2, Vector2>> raycastAABB(const Vector2& p, const Vector2& dir, const Vector2& topLeft, const Vector2& bottomRight);
		bool isPointOnAABB(const Vector2& p, const Vector2& topLeft, const Vector2& bottomRight);
		/// <summary>
		/// Rotate point 'p' around point 'center' by 'angle' degrees
		/// </summary>
		/// <param name="p">source point</param>
		/// <param name="center">center point</param>
		/// <param name="angle">rotate angle</param>
		/// <returns></returns>
		Vector2 rotate(const Vector2& p, const Vector2& center, const real& angle);
		/// <summary>
		/// Calculate the projection axis of ellipse in user-define direction.
		/// Return the maximum point in ellipse
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="direction"></param>
		/// <returns></returns>
		Vector2 calculateEllipseProjectionPoint(const real& a, const real& b, const Vector2& direction);
		Vector2 calculateCapsuleProjectionPoint(const real& width, const real& height, const Vector2& direction);
		Vector2 calculateSectorProjectionPoint(const real& startRadian, const real& spanRadian, const real& radius, const Vector2& direction);
		bool triangleContainsOrigin(const Vector2& a, const Vector2& b, const Vector2& c);
		bool isPointOnSameSide(const Vector2& edgePoint1, const Vector2& edgePoint2, const Vector2& refPoint, const Vector2 targetPoint);
	};
}
#endif
