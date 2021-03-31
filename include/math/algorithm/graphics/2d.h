#pragma once
#include "include/math/math.h"
#include "include/common/common.h"
namespace Physics2D
{
	class GraphicsAlgorithm2D
	{
	public:
		/// <summary>
		/// Judge point a,b,c collinear using triangle area method
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		static inline bool isCollinear(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			//triangle area = 0 then collinear
			return numberEqual(abs(Vector2::crossProduct(a - b, a - c)), 0);
		}
		/// <summary>
		/// Judge whether point c on line segment ab using line projection and set-union method
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		static inline bool isPointOnSegment(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			if (!isCollinear(a, b, c))
				return false;
			else
			{
				return (c.x <= max(a.x, b.x) && c.x >= min(a.x, b.x) &&
					c.y <= max(a.y, b.y) && c.y >= min(a.y, b.y));
			}
		}
		/// <summary>
		/// Calculate intersection point between line ab and line cd.
		/// Return if there is a actual point
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <param name="d"></param>
		/// <returns></returns>
		static std::optional<Vector2> lineSegmentIntersection(const Vector2& a, const Vector2& b, const Vector2& c, const Vector2& d)
		{
			Vector2 ab = b - a;
			Vector2 ac = c - a;
			Vector2 ad = d - a;
			Vector2 bc = c - b;
			Vector2 ba = a - b;
			Vector2 bd = d - b;
			number ab_length = ab.length();

			if (numberEqual(ab_length, 0.0f))
				return std::nullopt;

			number cc_proj = ab.cross(ac) / ab_length;
			number dd_proj = ba.cross(bd) / ab_length;
			number ad_proj = ad.dot(ab.normal());
			number bc_proj = bc.dot(ba.normal());
			number cproj_dproj = ab_length - ad_proj - bc_proj;
			number cp = cproj_dproj / (1 + (dd_proj / cc_proj));
			Vector2 bp = ba.normalize() * (bc_proj + cp);
			if (bp.length() == 0)
				return std::nullopt;

			Vector2 p = bp + b;
			number p_x = abs(p.x);
			number p_y = abs(p.y);

			if (p.x <= max(a.x, b.x) && p.x >= min(a.x, b.x) &&
				p.y <= max(a.y, b.y) && p.y >= min(a.y, b.y))
				return std::optional<Vector2>(p);
			else
				return std::nullopt;
		}
		static std::tuple<Vector2, number> calculateCircumcircle(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			
		}
		static std::tuple<Vector2, number> calculateInscribedCircle(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			
		}
		static bool isConvexPolygon(const std::vector<Vector2>& vertices)
		{
			
		}
		static std::vector<Vector2> polygonConvexHull(const std::vector<Vector2>& vertices)
		{
			
		}
		static Vector2 originToLineSegment(const Vector2& a, const Vector2& b)
		{
			//special cases
			if (a == b)
				return a;
			
			if((abs(a.x)  == 0 && abs(b.x) == 0)||(abs(a.y) == 0 && abs(b.y == 0)))
				return a.lengthSquare() > b.lengthSquare() ? b : a;
			else
			{
				Vector2 ab = b - a;
				Vector2 ao = ab.normal().dot(a * -1) * ab.normal();
				Vector2 op = a + ao;
				
				if((op.x <= max(a.x, b.x) && op.x >= min(a.x, b.x) &&
					op.y <= max(a.y, b.y) && op.y >= min(a.y, b.y)))
					return op;
				else
					return a.lengthSquare() > b.lengthSquare() ? b : a;
				
				
			}
		}
		/// <summary>
		/// Calculate shortest length between point p and ellipse ab,return the point on ellipse ab
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="p"></param>
		/// <returns></returns>
		static std::optional<Vector2> shortestLengthPointOfEllipse(const number& a, const number& b, const Vector2& p, const number& epsilon = 0.000001f)
		{
			if (a == 0 || b == 0)
				return std::nullopt;
			
			if(p.x == 0)
			{
				return p.y > 0 ? std::optional<Vector2>({ 0, b }) :
					std::optional<Vector2>({ 0, -b });
			}
			if (p.y == 0)
			{
				return p.x > 0 ? std::optional<Vector2>({ a, 0 }) :
					std::optional<Vector2>({ -a, 0 });
			}
			
			number x_left, x_right;
			number temp_x, temp_y;
			number t1_x, t1_y;
			Vector2 t0, t1;
			int sgn = p.y > 0 ? 1 : -1;
			if(p.x < 0)
			{
				x_left = -a;
				x_right = 0;
			}
			else
			{
				x_left = 0;
				x_right = a;
			}
			int iteration = 0;
			while(++iteration)
			{
				temp_x = (x_left + x_right) / 2;
				temp_y = sgn * sqrt(pow(b, 2) - pow(b / a, 2) * pow(temp_x, 2));
				Vector2 t0(temp_x, temp_y);
				t0.set(temp_x, temp_y);
				t1_x = temp_x + 1;
				t1_y = (pow(b, 2) - pow(b / a, 2) * t1_x * temp_x) / temp_y;
				t1.set(t1_x, t1_y);
				Vector2 t0t1 = t1 - t0;
				Vector2 t0p = p - t0;
				
				number result = t0t1.dot(t0p);
				if (abs(result) < epsilon)
					return std::optional<Vector2>(t0);
				
				if(result > 0)					// acute angle
					x_left = temp_x;
				else
					x_right = temp_x;			//obtuse angle
			}
		}
	};
}
