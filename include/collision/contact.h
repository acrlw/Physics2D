#pragma once
#include "include/math/math.h"
namespace Physics2D
{
	struct ContactInfo
	{
		Vector2 contactA;
		Vector2 contactB;
		bool isCollide = false;
		Vector2 penetration;
	};
	static inline std::optional<Vector2> lineSegmentIntersection(const Vector2& a, const Vector2& b, const Vector2& c, const Vector2& d)
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
		Vector2 bp = ba.normalize()* (bc_proj + cp);
		if (bp.length() == 0)
			return std::nullopt;
		
		Vector2 p = bp + b;
		number p_x = abs(p.x);
		number p_y = abs(p.y);
		
		if (p_x <= abs_max(a.x, b.x) && p_x >= abs_min(a.x, b.x) &&
			p_y <= abs_max(a.y, b.y) && p_y >= abs_min(a.y, b.y))
			return std::optional<Vector2>(p);
		else
			return std::nullopt;
	}
}