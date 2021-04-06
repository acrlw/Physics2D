#pragma once
#include "include/math/math.h"
#include "include/common/common.h"
namespace Physics2D
{
	class GeometryAlgorithm2D
	{
	public:
		/// <summary>
		/// Judge point a,b,c collinear using triangle area method
		/// </summary>
		/// <param name="a">point a</param>
		/// <param name="b">point b</param>
		/// <param name="c">point c</param>
		/// <returns></returns>
		static bool isCollinear(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			//triangle area = 0 then collinear
			return numberEqual(abs(Vector2::crossProduct(a - b, a - c)), 0);
		}
		/// <summary>
		/// Judge whether point c is on line segment ab using line projection and set-union method
		/// </summary>
		/// <param name="a">end of segment a</param>
		/// <param name="b">end of segment b</param>
		/// <param name="c">point c</param>
		/// <returns></returns>
		static bool isPointOnSegment(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			return !isCollinear(a, b, c) ? false : fuzzyIsPointOnSegment(a, b, c);
		}
		/// <summary>
		/// Judge whether point c is on line segment ab, given a,b,c is already collinear by calculating cross product
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		static bool fuzzyIsPointOnSegment(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			return (c.x <= max(a.x, b.x) && c.x >= min(a.x, b.x) &&
				c.y <= max(a.y, b.y) && c.y >= min(a.y, b.y));
		}
		/// <summary>
		/// Calculate intersected point between line ab and line cd.
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
		/// <summary>
		/// Calculate the center of circum-circle from triangle abc
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		static std::optional<Vector2> triangleCircumcenter(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			if (triangleArea(a, b, c) == 0)
				return std::nullopt;
			
			//2 * (x2 - x1) * x + 2 * (y2 - y1) y = x2 ^ 2 + y2 ^ 2 - x1 ^ 2 - y1 ^ 2;
			//2 * (x3 - x2) * x + 2 * (y3 - y2) y = x3 ^ 2 + y3 ^ 2 - x2 ^ 2 - y2 ^ 2;
			Matrix2x2 coef_mat{ 2 * (b.x - a.x), 2 * (c.x - b.x), 2 * (b.y - a.y), 2 * (c.y - b.y) };
			Vector2 constant{ b.lengthSquare() - a.lengthSquare(), c.lengthSquare() - b.lengthSquare() };
			return std::optional<Vector2>(coef_mat.invert().multiply(constant));
			
		}
		/// <summary>
		/// Calculate the center of inscribed-circle from triangle abc
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		static std::optional<Vector2> triangleIncenter(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			if (triangleArea(a, b, c) == 0)
				return std::nullopt;

			number ab = (b - a).length();
			number bc = (c - b).length();
			number ca = (a - c).length();
			Vector2 p = (ab * c + bc * a + ca * b) / (ab + bc + ca);
			return std::optional<Vector2>(p);
		}
		/// <summary>
		/// Calculate circum-circle given three points that can form a triangle
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		static std::optional<std::tuple<Vector2, number>> calculateCircumcircle(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			if (triangleArea(a, b, c) == 0)
				return std::nullopt;
			auto point = triangleCircumcenter(a, b, c);
			number radius = (point.value() - a).length();
			return std::make_tuple(point.value(), radius);
		}
		/// <summary>
		/// Calculate inscribed circle given three points that can form a triangle
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		static std::optional<std::tuple<Vector2, number>> calculateInscribedCircle(const Vector2& a, const Vector2& b, const Vector2& c)
		{
			number area = triangleArea(a, b, c);
			if (area == 0)
				return std::nullopt;

			number ab = (b - a).length();
			number bc = (c - b).length();
			number ca = (a - c).length();
			Vector2 p = (ab * c + bc * a + ca * b) / (ab + bc + ca);
			number radius = 2 * area / (ab + bc + ca);
			return std::make_tuple(p, radius);
		}
		/// <summary>
		/// Judge whether polygon is convex
		/// </summary>
		/// <param name="vertices"></param>
		/// <returns></returns>
		static bool isConvexPolygon(const std::vector<Vector2>& vertices)
		{
			if (vertices.size() == 4)
				return true;

			Vector2 ab, ac;
			for(uint16_t i = 0;i < vertices.size() - 1; i++)
			{
				ab = vertices[i + 1] - vertices[i];
				ac = i + 2 != vertices.size() ? vertices[i + 2] - vertices[i] : vertices[1] - vertices[i];
				if (Vector2::crossProduct(ab, ac) < 0)
					return false;
			}
			return true;
		}
		/// <summary>
		/// Convex hull algorithm: Graham Scan. Given a series of points, find the convex polygon that can contains all of these points.
		/// </summary>
		/// <param name="vertices"></param>
		/// <returns></returns>
		static std::vector<Vector2> grahamScan(const std::vector<Vector2>& vertices)
		{
			std::vector<Vector2> sort = vertices;
			std::vector<uint16_t> stack;
			
			std::sort(sort.begin(), sort.end(), [](const Vector2& a, const Vector2& b)
				{
					if (atan2f(a.y, a.x) != atan2f(b.y, b.x))
						return atan2f(a.y, a.x) < atan2(b.y, b.x);
					else
						return a.x < b.x;
				});

			uint16_t i, j, k;
			j = 1;
			k = 2;
			Vector2 ab, ac;
			stack.emplace_back(0);
			stack.emplace_back(1);
			while(true)
			{
				i = stack[stack.size() - 2];
				j = stack[stack.size() - 1];
				if (j == 0)
					break;
				
				if (k >= sort.size())
					k = 0;
				
				ab = sort[j] - sort[i];
				ac = sort[k] - sort[i];
				if(ab.cross(ac) < 0)
					stack.pop_back();
				stack.emplace_back(k);
				k++;
			}
			std::vector<Vector2> result;
			for(auto index: stack)
				result.emplace_back(sort[index]);
			
			return result;
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
		static Vector2 triangleCentroid(const Vector2& a1, const Vector2& a2, const Vector2& a3)
		{
			return Vector2(a1 + a2 + a3) / 3;
		}
		/// <summary>
		/// Calculate the area of triangle use cross product
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
		/// Calculate mass center of 'convex' polygon
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
					Vector2 p = triangleCentroid(vertices[p_a], vertices[p_b], vertices[p_c]);
					pos += p * a;
					area += a;
				}
				pos /= area;
				return pos;
			}
		}
		/// <summary>
		/// Calculate two points on line segment and ellipse respectively. The length of two points is the shortest distance of line segment and ellipse
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
				//calculate tangent line
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

				//Judge which quadrant does nearest point stay
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
		/// Calculate point on line segment ab, if point 'p' can cast ray in 'dir' direction on line segment ab.
		/// Algorithm from wikipedia 'Line-line intersection'
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
		/// <summary>
		/// Rotate point 'p' around point 'center' by 'angle' degrees
		/// </summary>
		/// <param name="p">source point</param>
		/// <param name="center">center point</param>
		/// <param name="angle">rotate angle</param>
		/// <returns></returns>
		static Vector2 rotate(const Vector2& p, const Vector2& center, const number& angle)
		{
			return Matrix2x2(angle).multiply(p - center) + center;
		}
	};
}
