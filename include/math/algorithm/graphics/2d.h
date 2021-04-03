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
		/// Judge whether point c is on line segment ab using line projection and set-union method
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
				return fuzzyIsPointOnSegment(a, b, c);
			}
		}
		/// <summary>
		/// Judge whether point c is on line segment ab, given a,b,c is already collinear by calculating cross product
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		static inline bool fuzzyIsPointOnSegment(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			return (c.x <= max(a.x, b.x) && c.x >= min(a.x, b.x) &&
				c.y <= max(a.y, b.y) && c.y >= min(a.y, b.y));
		}
		/// <summary>
		/// Calculate intersection point between line ab and line cd.
		/// Return if there is a actual intersected point
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

			const number cc_proj = ab.cross(ac) / ab_length;
			const number dd_proj = ba.cross(bd) / ab_length;
			const number ad_proj = ad.dot(ab.normal());
			const number bc_proj = bc.dot(ba.normal());
			const number cproj_dproj = ab_length - ad_proj - bc_proj;
			const number denominator = (1 + (dd_proj / cc_proj));
			if (numberEqual(denominator, 0.0f))
				return std::nullopt;

			const number cp = cproj_dproj / denominator;
			const Vector2 bp = ba.normalize() * (bc_proj + cp);
			if (numberEqual(bp.length(), 0))
				return std::nullopt;

			Vector2 p = bp + b;
			number p_x = abs(p.x);
			number p_y = abs(p.y);

			return fuzzyIsPointOnSegment(a, b, p) ? std::optional<Vector2>(p)
				: std::nullopt;

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
		/// <summary>
		/// calculate point on line segment ab that is the shortest length to point p
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		static Vector2 pointToLineSegment(const Vector2& a, const Vector2& b, const Vector2& p)
		{
			//special cases
			if (a == b)
				return {};
			
			Vector2 ap = p - a;
			Vector2 ab_normal = (b - a).normal();
			Vector2 ap_proj = ab_normal.dot(ap) * ab_normal;
			Vector2 op_proj = a + ap_proj;
			
			if(fuzzyIsPointOnSegment(a, b, op_proj))
				return op_proj;
			else
				return (p - a).lengthSquare() > (p - b).lengthSquare() ? b : a;
		}
		/// <summary>
		/// Calculate point on ellipse that is the shortest length to point p.
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="p"></param>
		/// <returns></returns>
		static Vector2 shortestLengthPointOfEllipse(const number& a, const number& b, const Vector2& p, const number& epsilon = 0.00000001f)
		{
			if (a == 0 || b == 0)
				return {};
			
			if(p.x == 0)
			{
				return p.y > 0 ? Vector2{ 0, b }
					: Vector2{ 0, -b };
			}
			if (p.y == 0)
			{
				return p.x > 0 ? Vector2{ a, 0 }
				: Vector2{ -a, 0 };
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
					return t0;
				
				if(result > 0)					// acute angle
					x_left = temp_x;
				else
					x_right = temp_x;			//obtuse angle
			}
		}
		static Vector2 triangleGravityPoint(const Vector2& a1, const Vector2& a2, const Vector2& a3)
		{
			return Vector2(a1 + a2 + a3) / 3;
		}
		/// <summary>
		/// calculate the area of triangle use cross product
		/// </summary>
		/// <param name="a1"></param>
		/// <param name="a2"></param>
		/// <param name="a3"></param>
		/// <returns></returns>
		static number triangleArea(const Vector2& a1, const Vector2& a2, const Vector2& a3)
		{
			return abs(Vector2::crossProduct(a1 - a2, a1 - a3)) / 2;
		}
		/// <summary>
		/// calculate mass center of 'convex' polygon
		/// </summary>
		/// <param name="vertices"></param>
		/// <returns></returns>
		static Vector2 calculateCenter(const std::vector<Vector2>& vertices)
		{
			if (vertices.size() >= 4)
			{
				Vector2 pos;
				number area = 0;
				size_t p_a, p_b, p_c;
				p_a = 0, p_b = 0, p_c = 0;
				for (size_t i = 0; i < vertices.size() - 1; i++)
				{
					p_b = i + 1;
					p_c = i + 2;
					if (p_b == vertices.size() - 2)
						break;
					number a = triangleArea(vertices[p_a], vertices[p_b], vertices[p_c]);
					Vector2 p = triangleGravityPoint(vertices[p_a], vertices[p_b], vertices[p_c]);
					pos += p * a;
					area += a;
				}
				pos /= area;
				return pos;
			}
		}
		/// <summary>
		/// calculate two points on line segment and ellipse respectively. The length of two points is the shortest distance of line segment and ellipse
		/// </summary>
		/// <param name="a">major axis a</param>
		/// <param name="b">minor axis b</param>
		/// <param name="p1">line segment point 1</param>
		/// <param name="p2">line segment point 2</param>
		/// <returns></returns>
		static std::tuple<Vector2, Vector2> shortestLengthLineSegmentEllipse(const number& a, const number& b, const Vector2& p1, const Vector2& p2)
		{
			Vector2 p_line;
			Vector2 p_ellipse;
			if(numberEqual(p1.y, p2.y))
			{
				if(!((p1.x > 0 && p2.x > 0)||(p1.x < 0 && p2.x < 0))) // different quadrant
				{
					p_ellipse.set(0, p1.y > 0 ? b : -b);
					p_line.set(0, p1.y);
				}
				else
				{
					p_line.set(abs(p1.x) > abs(p2.x) ? p2.x : p1.x, p1.y);
					p_ellipse = shortestLengthPointOfEllipse(a, b, p_line);
				}
			}
			else if (numberEqual(p1.x, p2.x))
			{
				if (!((p1.y > 0 && p2.y > 0) || (p1.y < 0 && p2.y < 0))) // different quadrant
				{
					p_ellipse.set(p1.x > 0 ? a : -a, 0);
					p_line.set(p1.x, 0);
				}
				else
				{
					p_line.set(p1.x, abs(p1.y) > abs(p2.y) ? p2.y : p1.y);
					p_ellipse = shortestLengthPointOfEllipse(a, b, p_line);
				}
			}
			else
			{
				
				number k = (p2.y - p1.y) / (p2.x - p1.x);
				number k2 = k * k;
				number a2 = a * a;
				number b2 = b * b;
				int sgn = k > 0 ? 1 : -1;
				number f_x2 = (k2 * a2 * a2 / b2) / (1 + a2 * k2 / b2);
				number f_y2 = b2 - b2 * f_x2 / a2;
				number f_x = sqrt(f_x2);
				number f_y = sqrt(f_y2);
				Vector2 f;
				Vector2 p1p2 = (p2 - p1).normal();
				
				{
					Vector2 f_arr[4];
					f_arr[0].set(f_x, f_y);
					f_arr[1].set(-f_x, f_y);
					f_arr[2].set(-f_x, -f_y);
					f_arr[3].set(f_x, -f_y);
					number min = Vector2::crossProduct(p1p2, f_arr[0] - p1);
					for (int i = 1; i < 4; i++)
					{
						number value = Vector2::crossProduct(p1p2, f_arr[i] - p1);
						if(min > value)
						{
							f = f_arr[i];
							min = value;
						}
					}
				}

				Vector2 p1f = f - p1;
				Vector2 p1_fp = p1p2 * p1p2.dot(p1f);
				Vector2 f_proj = p1 + p1_fp;

				if(fuzzyIsPointOnSegment(a, b, f_proj))
				{
					p_ellipse = f;
					p_line = f_proj;
				}
				else
				{
					Vector2 p1_p = shortestLengthPointOfEllipse(a, b, p1);
					Vector2 p2_p = shortestLengthPointOfEllipse(a, b, p2);
					if((p1 - p1_p).lengthSquare() > (p2 - p2_p).lengthSquare())
					{
						p_ellipse = p2_p;
						p_line = p2;
					}
					else
					{
						p_ellipse = p1_p;
						p_line = p1;
					}
				}
			}
			return std::make_tuple(p_ellipse, p_line);
		}
		/// <summary>
		/// Calculate point on line segment ab, if point 'p' can cast ray in 'dir' direction on line segment ab
		/// </summary>
		/// <param name="p">ray start point</param>
		/// <param name="dir">ray direction</param>
		/// <param name="a">line segment point a</param>
		/// <param name="b">line segment point b</param>
		/// <returns></returns>
		static std::optional<Vector2> raycast(const Vector2& p, const Vector2& dir, const Vector2& a, const Vector2& b)
		{
			const number denominator = (p.x - dir.x) * (a.y - b.y) - (p.y - dir.y) * (a.x - b.x);
			
			if (numberEqual(denominator, 0))
				return std::nullopt;

			const number t = ((p.x - a.x) * (a.y - b.y) - (p.y - a.y) * (a.x - b.x)) / denominator;
			const number u = ((dir.x - p.x) * (p.y - a.y) - (dir.y - p.y) * (p.x - a.x)) / denominator;
			if (t >= 0 && u <= 1.0 && u >= 0)
			{
				return std::optional<Vector2>({p.x + t * (dir.x - p.x), p.y + t * (dir.y - p.y) });
			}
			else
				return std::nullopt;
		}
	};
}
