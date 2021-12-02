#include "../../../include/geometry/algorithm/2d.h"

namespace Physics2D
{
	std::vector<Vector2> GeometryAlgorithm2D::Clipper::sutherlandHodgmentPolygonClipping(const std::vector<Vector2>& polygon, const std::vector<Vector2>& clipRegion)
	{
		std::vector<Vector2> result = polygon;

		for (size_t i = 0; i < clipRegion.size() - 1; i++)
		{
			Vector2 clipPoint1 = clipRegion[i];
			Vector2 clipPoint2 = clipRegion[i + 1];
			Vector2 clipDirectionPoint = i + 2 == clipRegion.size() ? clipRegion[1] : clipRegion[i + 2];
			std::vector<int8_t> testResults;
			testResults.reserve(polygon.size());

			for (size_t j = 0; j < result.size(); j++)
			{
				bool res = GeometryAlgorithm2D::isPointOnSameSide(clipPoint1, clipPoint2, clipDirectionPoint, result[j]);
				testResults.emplace_back(res ? 1 : -1);
			}
			std::vector<Vector2> newPolygon;
			newPolygon.reserve(result.size());

			for (size_t j = 1; j < testResults.size(); j++)
			{
				bool lastInside = testResults[j - 1] == 1 ? true : false;
				bool currentInside = testResults[j] == 1 ? true : false;
				//last inside and current outside
				if (lastInside && !currentInside)
				{
					//push last point
					newPolygon.emplace_back(result[j - 1]);
					//push intersection point
					Vector2 p = GeometryAlgorithm2D::lineIntersection(clipPoint1, clipPoint2, result[j - 1], result[j]);
					newPolygon.emplace_back(p);
				}
				//last outside and current inside
				if (!lastInside && currentInside)
				{
					//push intersection point first
					Vector2 p = GeometryAlgorithm2D::lineIntersection(clipPoint1, clipPoint2, result[j - 1], result[j]);
					newPolygon.emplace_back(p);
				}
				//last outside and current outside
				if (!lastInside && !currentInside)
				{
					//do nothing
				}
				if (lastInside && currentInside)
				{
					//push last vertex
					newPolygon.emplace_back(result[j - 1]);
				}
			}
			result = newPolygon;
			result.emplace_back(result[0]);
		}
		return result;
	}
	bool GeometryAlgorithm2D::isCollinear(const Vector2& a, const Vector2& b, const Vector2& c)
	{
		//triangle area = 0 then collinear
		return realEqual(std::fabs((a - b).cross(a - c)), 0);
	}

	bool GeometryAlgorithm2D::isPointOnSegment(const Vector2& a, const Vector2& b, const Vector2& c)
	{
		return !isCollinear(a, b, c) ? false : fuzzyIsCollinear(a, b, c);
	}

	bool GeometryAlgorithm2D::fuzzyIsPointOnSegment(const Vector2& a, const Vector2& b, const Vector2& c,
	                                                const real& epsilon)
	{
		return fuzzyRealEqual(pointToLineSegment(a, b, c).lengthSquare(), epsilon);
	}
	bool GeometryAlgorithm2D::fuzzyIsCollinear(const Vector2& a, const Vector2& b, const Vector2& c)
	{
		return (c.x <= Math::max(a.x, b.x) && c.x >= Math::min(a.x, b.x) &&
			c.y <= Math::max(a.y, b.y) && c.y >= Math::min(a.y, b.y));
	}
	std::optional<Vector2> GeometryAlgorithm2D::lineSegmentIntersection(const Vector2& a, const Vector2& b,
	                                                                    const Vector2& c, const Vector2& d)
	{
		const Vector2 ab = b - a;
		const Vector2 ac = c - a;
		const Vector2 ad = d - a;
		const Vector2 bc = c - b;
		Vector2 ba = a - b;
		const Vector2 bd = d - b;
		const real ab_length = ab.length();

		if (realEqual(ab_length, 0.0))
		{
			if (fuzzyIsCollinear(c, d, a))
				return std::optional(a);
			return std::nullopt;
		}
		const real ab_length_inv = 1 / ab_length;
		const real cc_proj = ab.cross(ac) * ab_length_inv;
		const real dd_proj = ba.cross(bd) * ab_length_inv;
		const real ad_proj = ad.dot(ab.normal());
		const real bc_proj = bc.dot(ba.normal());
		const real cproj_dproj = ab_length - ad_proj - bc_proj;

		if (realEqual(cc_proj, 0.0))
			return std::nullopt;

		const real denominator = (1 + (dd_proj / cc_proj));
		if (realEqual(denominator, 0.0))
			return std::nullopt;

		const real cp = cproj_dproj / denominator;
		const Vector2 bp = ba.normalize() * (bc_proj + cp);
		if (realEqual(bp.length(), 0))
			return std::nullopt;

		Vector2 p = bp + b;

		return (fuzzyIsCollinear(a, b, p) && fuzzyIsCollinear(d, c, p))
			       ? std::optional(p)
			       : std::nullopt;
	}

	Vector2 GeometryAlgorithm2D::lineIntersection(const Vector2& p1, const Vector2& p2, const Vector2& q1,
		const Vector2& q2)
	{
		const real d = (p1.x - p2.x) * (q1.y - q2.y) - (p1.y - p2.y) * (q1.x - q2.x);
		if (realEqual(d, 0))
			return Vector2();
		const real x = ((p1.x * p2.y - p1.y * p2.x) * (q1.x - q2.x) - (q1.x * q2.y - q1.y * q2.x) * (p1.x - p2.x)) / d;
		const real y = ((p1.x * p2.y - p1.y * p2.x) * (q1.y - q2.y) - (q1.x * q2.y - q1.y * q2.x) * (p1.y - p2.y)) / d;
		return Vector2(x, y);
	}

	std::optional<Vector2> GeometryAlgorithm2D::triangleCircumcenter(const Vector2& a, const Vector2& b,
	                                                                 const Vector2& c)
	{
		if (realEqual(triangleArea(a, b, c), 0))
			return std::nullopt;

		//2 * (x2 - x1) * x + 2 * (y2 - y1) y = x2 ^ 2 + y2 ^ 2 - x1 ^ 2 - y1 ^ 2;
		//2 * (x3 - x2) * x + 2 * (y3 - y2) y = x3 ^ 2 + y3 ^ 2 - x2 ^ 2 - y2 ^ 2;
		Matrix2x2 coef_mat{2.0f * (b.x - a.x), 2.0f * (c.x - b.x), 2.0f * (b.y - a.y), 2 * (c.y - b.y)};
		const Vector2 constant{b.lengthSquare() - a.lengthSquare(), c.lengthSquare() - b.lengthSquare()};
		return std::optional(coef_mat.invert().multiply(constant));
	}

	std::optional<Vector2> GeometryAlgorithm2D::triangleIncenter(const Vector2& a, const Vector2& b, const Vector2& c)
	{
		if (triangleArea(a, b, c) == 0)
			return std::nullopt;

		const real ab = (b - a).length();
		const real bc = (c - b).length();
		const real ca = (a - c).length();
		Vector2 p = (ab * c + bc * a + ca * b) / (ab + bc + ca);
		return std::optional(p);
	}

	std::optional<std::tuple<Vector2, real>> GeometryAlgorithm2D::calculateCircumcircle(
		const Vector2& a, const Vector2& b, const Vector2& c)
	{
		if (triangleArea(a, b, c) == 0)
			return std::nullopt;
		auto point = triangleCircumcenter(a, b, c);
		real radius = (point.value() - a).length();
		return std::make_tuple(point.value(), radius);
	}

	std::optional<std::tuple<Vector2, real>> GeometryAlgorithm2D::calculateInscribedCircle(
		const Vector2& a, const Vector2& b, const Vector2& c)
	{
		const real area = triangleArea(a, b, c);
		if (area == 0)
			return std::nullopt;

		const real ab = (b - a).length();
		const real bc = (c - b).length();
		const real ca = (a - c).length();
		Vector2 p = (ab * c + bc * a + ca * b) / (ab + bc + ca);
		real radius = 2.0f * area / (ab + bc + ca);
		return std::make_tuple(p, radius);
	}

	bool GeometryAlgorithm2D::isConvexPolygon(const std::vector<Vector2>& vertices)
	{
		if (vertices.size() == 4)
			return true;

		for (size_t i = 0; i < vertices.size() - 1; i++)
		{
			Vector2 ab = vertices[i + 1] - vertices[i];
			Vector2 ac = i + 2 != vertices.size() ? vertices[i + 2] - vertices[i] : vertices[1] - vertices[i];
			if (Vector2::crossProduct(ab, ac) < 0)
				return false;
		}
		return true;
	}

	std::vector<Vector2> GeometryAlgorithm2D::grahamScan(const std::vector<Vector2>& vertices)
	{
		std::vector<Vector2> sort = vertices;
		std::vector<uint16_t> stack;

		std::sort(sort.begin(), sort.end(), [](const Vector2& a, const Vector2& b)
		{
			if (atan2l(a.y, a.x) != atan2l(b.y, b.x))
				return atan2l(a.y, a.x) < atan2l(b.y, b.x);
			return a.x < b.x;
		});

		uint16_t k = 2;
		stack.emplace_back(0);
		stack.emplace_back(1);
		while (true)
		{
			uint16_t i = stack[stack.size() - 2];
			uint16_t j = stack[stack.size() - 1];
			if (j == 0)
				break;

			if (k >= sort.size())
				k = 0;

			Vector2 ab = sort[j] - sort[i];
			Vector2 ac = sort[k] - sort[i];
			if (ab.cross(ac) < 0)
				stack.pop_back();
			stack.emplace_back(k);
			k++;
		}
		std::vector<Vector2> result;
		result.reserve(stack.size());
		for (const auto index : stack)
			result.emplace_back(sort[index]);

		return result;
	}


	Vector2 GeometryAlgorithm2D::pointToLineSegment(const Vector2& a, const Vector2& b, const Vector2& p)
	{
		//special cases
		if (a == b)
			return {};

		if (isCollinear(a, b, p))
			return p;

		const Vector2 ap = p - a;
		const Vector2 ab_normal = (b - a).normal();
		const Vector2 ap_proj = ab_normal.dot(ap) * ab_normal;
		Vector2 op_proj = a + ap_proj;

		if (fuzzyIsCollinear(a, b, op_proj))
			return op_proj;
		return (p - a).lengthSquare() > (p - b).lengthSquare() ? b : a;
	}

	Vector2 GeometryAlgorithm2D::shortestLengthPointOfEllipse(const real& a, const real& b, const Vector2& p,
	                                                          const real& epsilon)
	{
		if (realEqual(a, 0) || realEqual(b, 0))
			return {};

		if (realEqual(p.x, 0))
		{
			return p.y > 0
				       ? Vector2{0, b}
				       : Vector2{0, -b};
		}
		if (realEqual(p.y, 0))
		{
			return p.x > 0
				       ? Vector2{a, 0}
				       : Vector2{-a, 0};
		}

		real x_left, x_right;
		Vector2 t0, t1;
		const int sgn = p.y > 0 ? 1 : -1;
		if (p.x < 0)
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
		while (++iteration)
		{
			real temp_x = (x_left + x_right) * 0.5f;
			real temp_y = sgn * sqrt(pow(b, 2.0f) - pow(b / a, 2.0f) * pow(temp_x, 2.0f));
			Vector2 t0(temp_x, temp_y);
			t0.set(temp_x, temp_y);
			real t1_x = temp_x + 1;
			real t1_y = (pow(b, 2.0f) - pow(b / a, 2.0f) * t1_x * temp_x) / temp_y;
			t1.set(t1_x, t1_y);
			Vector2 t0t1 = t1 - t0;
			Vector2 t0p = p - t0;

			const real result = t0t1.dot(t0p);
			if (std::fabs(result) < epsilon)
				break;

			if (result > 0) // acute angle
				x_left = temp_x;
			else
				x_right = temp_x; //obtuse angle
		}
		return t0;
	}

	Vector2 GeometryAlgorithm2D::triangleCentroid(const Vector2& a1, const Vector2& a2, const Vector2& a3)
	{
		return Vector2(a1 + a2 + a3) / 3.0f;
	}

	real GeometryAlgorithm2D::triangleArea(const Vector2& a1, const Vector2& a2, const Vector2& a3)
	{
		return std::fabs(Vector2::crossProduct(a1 - a2, a1 - a3)) / 2.0f;
	}

	Vector2 GeometryAlgorithm2D::calculateCenter(const std::vector<Vector2>& vertices)
	{
		if (vertices.size() >= 4)
		{
			Vector2 pos;
			real area = 0;
			size_t p_a, p_b, p_c;
			p_a = 0, p_b = 0, p_c = 0;
			for (size_t i = 0; i < vertices.size() - 1; i++)
			{
				p_b = i + 1;
				p_c = i + 2;
				if (p_b == vertices.size() - 2)
					break;
				real a = triangleArea(vertices[p_a], vertices[p_b], vertices[p_c]);
				Vector2 p = triangleCentroid(vertices[p_a], vertices[p_b], vertices[p_c]);
				pos += p * a;
				area += a;
			}
			pos /= area;
			return pos;
		}
			return Vector2();
	}

	std::tuple<Vector2, Vector2> GeometryAlgorithm2D::shortestLengthLineSegmentEllipse(
		const real& a, const real& b, const Vector2& p1, const Vector2& p2)
	{
		Vector2 p_line;
		Vector2 p_ellipse;
		if (realEqual(p1.y, p2.y))
		{
			if (!((p1.x > 0 && p2.x > 0) || (p1.x < 0 && p2.x < 0))) // different quadrant
			{
				p_ellipse.set(0, p1.y > 0 ? b : -b);
				p_line.set(0, p1.y);
			}
			else
			{
				p_line.set(std::fabs(p1.x) > std::fabs(p2.x) ? p2.x : p1.x, p1.y);
				p_ellipse = shortestLengthPointOfEllipse(a, b, p_line);
			}
		}
		else if (realEqual(p1.x, p2.x))
		{
			if (!((p1.y > 0 && p2.y > 0) || (p1.y < 0 && p2.y < 0))) // different quadrant
			{
				p_ellipse.set(p1.x > 0 ? a : -a, 0);
				p_line.set(p1.x, 0);
			}
			else
			{
				p_line.set(p1.x, std::fabs(p1.y) > std::fabs(p2.y) ? p2.y : p1.y);
				p_ellipse = shortestLengthPointOfEllipse(a, b, p_line);
			}
		}
		else
		{
			//calculate tangent line
			const real k = (p2.y - p1.y) / (p2.x - p1.x);
			const real k2 = k * k;
			const real a2 = a * a;
			const real b2 = b * b;
			const real f_x2 = (k2 * a2 * a2 / b2) / (1 + a2 * k2 / b2);
			const real f_y2 = b2 - b2 * f_x2 / a2;
			const real f_x = sqrt(f_x2);
			const real f_y = sqrt(f_y2);
			Vector2 f;
			const Vector2 p1p2 = (p2 - p1).normal();

			//Check which quadrant does nearest point fall in
			{
				Vector2 f_arr[4];
				f_arr[0].set(f_x, f_y);
				f_arr[1].set(-f_x, f_y);
				f_arr[2].set(-f_x, -f_y);
				f_arr[3].set(f_x, -f_y);
				real min = Vector2::crossProduct(p1p2, f_arr[0] - p1);
				for (int i = 1; i < 4; i++)
				{
					const real value = Vector2::crossProduct(p1p2, f_arr[i] - p1);
					if (min > value)
					{
						f = f_arr[i];
						min = value;
					}
				}
			}

			const Vector2 p1f = f - p1;
			const Vector2 p1_fp = p1p2 * p1p2.dot(p1f);
			const Vector2 f_proj = p1 + p1_fp;

			if (fuzzyIsCollinear(a, b, f_proj))
			{
				p_ellipse = f;
				p_line = f_proj;
			}
			else
			{
				const Vector2 p1_p = shortestLengthPointOfEllipse(a, b, p1);
				const Vector2 p2_p = shortestLengthPointOfEllipse(a, b, p2);
				if ((p1 - p1_p).lengthSquare() > (p2 - p2_p).lengthSquare())
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

	std::optional<Vector2> GeometryAlgorithm2D::raycast(const Vector2& p, const Vector2& dir, const Vector2& a,
	                                                    const Vector2& b)
	{
		const real denominator = (p.x - dir.x) * (a.y - b.y) - (p.y - dir.y) * (a.x - b.x);

		if (realEqual(denominator, 0))
			return std::nullopt;

		const real t = ((p.x - a.x) * (a.y - b.y) - (p.y - a.y) * (a.x - b.x)) / denominator;
		const real u = ((dir.x - p.x) * (p.y - a.y) - (dir.y - p.y) * (p.x - a.x)) / denominator;
		if (t >= 0 && u <= 1.0 && u >= 0)
			return std::optional<Vector2>({p.x + t * (dir.x - p.x), p.y + t * (dir.y - p.y)});
		return std::nullopt;
	}

	std::optional<std::pair<Vector2, Vector2>> GeometryAlgorithm2D::raycastAABB(const Vector2& p, const Vector2& dir, const Vector2& topLeft, const Vector2& bottomRight)
	{
		const real xmin = topLeft.x;
		const real ymin = bottomRight.y;
		const real xmax = bottomRight.x;
		const real ymax = topLeft.y;
		real txmin, txmax, tymin, tymax;
		real txenter, txexit, tyenter, tyexit;
		real tenter, texit;
		if(realEqual(dir.x, 0) && !realEqual(dir.y, 0))
		{
			tymin = (ymin - p.y) / dir.y;
			tymax = (ymax - p.y) / dir.y;
			tenter = Math::min(tymin, tymax);
			texit = Math::max(tymin, tymax);
		}
		else if (!realEqual(dir.x, 0) && realEqual(dir.y, 0))
		{

			txmin = (xmin - p.x) / dir.x;
			txmax = (xmax - p.x) / dir.x;
			tenter = Math::min(txmin, txmax);
			texit = Math::max(txmin, txmax);
		}
		else
		{
			txmin = (xmin - p.x) / dir.x;
			txmax = (xmax - p.x) / dir.x;
			tymin = (ymin - p.y) / dir.y;
			tymax = (ymax - p.y) / dir.y;
			txenter = Math::min(txmin, txmax);
			txexit = Math::max(txmin, txmax);
			tyenter = Math::min(tymin, tymax);
			tyexit = Math::max(tymin, tymax);
			tenter = Math::max(txenter, tyenter);
			texit = Math::min(txexit, tyexit);
		}
		if (tenter < 0 && texit < 0)
			return std::nullopt;
		Vector2 enter = p + tenter * dir;
		Vector2 exit = p + texit * dir;

		return std::make_pair(enter, exit);

	}

	bool GeometryAlgorithm2D::isPointOnAABB(const Vector2& p, const Vector2& topLeft, const Vector2& bottomRight)
	{
		return Math::isInRange(p.x, topLeft.x, bottomRight.x) &&
			Math::isInRange(p.y, bottomRight.y, topLeft.y);
	}

	Vector2 GeometryAlgorithm2D::rotate(const Vector2& p, const Vector2& center, const real& angle)
	{
		return Matrix2x2(angle).multiply(p - center) + center;
	}

	Vector2 GeometryAlgorithm2D::calculateEllipseProjectionPoint(const real& a, const real& b, const Vector2& direction)
	{
		Vector2 target;
		if (realEqual(direction.x, 0))
		{
			const int sgn = direction.y < 0 ? -1 : 1;
			target.set(0, sgn * b);
		}
		else if (realEqual(direction.y, 0))
		{
			const int sgn = direction.x < 0 ? -1 : 1;
			target.set(sgn * a, 0);
		}
		else
		{
			const real k = direction.y / direction.x;
			//line offset constant d
			const real a2 = pow(a, 2.0f);
			const real b2 = pow(b, 2.0f);
			const real k2 = pow(k, 2.0f);
			real d = sqrt((a2 + b2 * k2) / k2);
			if (Vector2::dotProduct(Vector2(0, d), direction) < 0)
				d = d * -1;
			const real x1 = k * d - (b2 * k2 * k * d) / (a2 + b2 * k2);
			const real y1 = (b2 * k2 * d) / (a2 + b2 * k2);
			target.set(x1, y1);
		}
		return target;
	}
	Vector2 GeometryAlgorithm2D::calculateCapsuleProjectionPoint(const real& width, const real& height,
	                                                             const Vector2& direction)
	{
		Vector2 target;
		if (width >= height) // Horizontal
		{
			real radius = height / 2.0f;
			real offset = direction.x >= 0 ? width / 2 - radius : radius - width / 2;
			target = direction.normal() * radius;
			target.x += offset;
		}
		else // Vertical
		{
			real radius = width / 2.0f;
			real offset = direction.y >= 0 ? height / 2 - radius : radius - height / 2;
			target = direction.normal() * radius;
			target.y += offset;
		}
		return target;
	}

	Vector2 GeometryAlgorithm2D::calculateSectorProjectionPoint(const real& startRadian, const real& spanRadian,
		const real& radius, const Vector2& direction)
	{
		Vector2 result;
		auto clampRadian = [](const real& radian)
		{
			real result = radian;
			result -= std::floor(result / Constant::DoublePi) * Constant::DoublePi;
			if (result < 0)
				result += Constant::DoublePi;
			return result;
		};

		const real clampStart = clampRadian(startRadian);
		const real clampEnd = clampRadian(startRadian + spanRadian);
		const real originStart = clampRadian(startRadian - Constant::HalfPi);
		const real originEnd = clampRadian(startRadian + spanRadian + Constant::HalfPi);
		
		const real originTheta = direction.theta();
		real theta = clampRadian(originTheta);

		if(originStart > originEnd)
		{
			//does not fall in zero area
			if (!Math::isInRange(theta, originEnd, originStart))
			{
				if(theta > originStart)
					theta = originTheta;
				
				//clamp theta to sector area
				
				result = Matrix2x2(Math::clamp(theta, clampStart, clampEnd)).multiply(Vector2{ 1, 0 }) * radius;
			}
		}
		if(originStart < originEnd)
		{
			if (Math::isInRange(originTheta, originStart, originEnd))
			{
				//clamp theta to sector area
				result = Matrix2x2(Math::clamp(theta, clampStart, clampEnd)).multiply(Vector2{ 1, 0 }) * radius;
			}
		}
		//special case for half circle
		if(fuzzyRealEqual(originStart, originEnd))
		{
			if(!fuzzyRealEqual(theta, originStart))
			{
				if (theta > originStart)
					theta = originTheta;
				if (clampStart > clampEnd)
					result = Matrix2x2(Math::clamp(theta, clampStart - Constant::DoublePi, clampEnd)).multiply(Vector2{ 1, 0 }) * radius;
				else
					result = Matrix2x2(Math::clamp(theta, clampStart, clampEnd)).multiply(Vector2{ 1, 0 }) * radius;
			}
		}
		return result;
	}

	bool GeometryAlgorithm2D::triangleContainsOrigin(const Vector2& a, const Vector2& b, const Vector2& c)
	{
		real ra = (b - a).cross(-a);
		real rb = (c - b).cross(-b);
		real rc = (a - c).cross(-c);
		return ra >= 0 && rb >= 0 && rc >= 0
		|| ra <= 0 && rb <= 0 && rc <= 0;
	}
	bool GeometryAlgorithm2D::isPointOnSameSide(const Vector2& edgePoint1, const Vector2& edgePoint2, const Vector2& refPoint, const Vector2 targetPoint)
	{
		Vector2 u = edgePoint2 - edgePoint1;
		Vector2 v = refPoint - edgePoint1;
		Vector2 w = targetPoint - edgePoint1;
		real d1 = u.cross(v);
		real d2 = u.cross(w);
		//same side or on the edge
		return Math::sign(d1) == Math::sign(d2) ;
	}
}
