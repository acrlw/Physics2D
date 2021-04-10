#ifndef PHYSICS2D_MATH_H
#define PHYSICS2D_MATH_H
#include "include/common/common.h"

namespace Physics2D
{
	//trigonometric function
	inline real sinx(const real& x)
	{
		return std::sin(x);
	}
	inline real cosx(const real& x)
	{
		return std::cos(x);
	}
	inline real tanx(const real& x)
	{
		return std::tan(x);
	}
	inline real arcsinx(const real& x)
	{
		return std::asin(x);
	}
	inline real arccosx(const real& x)
	{
		return std::acos(x);
	}
	inline real arctanx(const real& x)
	{
		return std::atan(x);
	}
	inline real max(const real& a, const real& b)
	{
		return std::max(a, b);
	}
	inline real min(const real& a, const real& b)
	{
		return std::min(a, b);
	}
	inline real abs_max(const real& a, const real& b)
	{
		return std::max(std::fabs(a), std::fabs(b));
	}
	inline real abs_min(const real& a, const real& b)
	{
		return std::min(std::fabs(a), std::fabs(b));
	}
	//other
	inline int sign(const real& num)
	{
		return num > 0 ? 1 : -1;
	}
	inline real clamp(const real& num, const real& low, const real& high)
	{
		return std::clamp(num, low, high);
	}
	//basic real utility
	inline void realSwap(real& lhs, real& rhs)
	{
		const real temp = lhs;
		lhs = rhs;
		rhs = temp;
	}
	inline bool realEqual(const real& lhs, const real& rhs)
	{
		return abs(lhs - rhs) < EPSILON;
	}
	inline real angleToRadian(const real& angle)
	{
		return angle * (180.0f / PI);
	}
	inline real radianToAngle(const real& radian)
	{
		return radian * (PI / 180.0f);
	}
	template <typename T, size_t iterations = 2> inline T fastInverseSqrt(T x) {
		typedef typename std::conditional<sizeof(T) == 8, std::int64_t, std::int32_t>::type Tint;
		T y = x;
		T x2 = y * 0.5;
		Tint i = *(Tint*)&y;
		i = (sizeof(T) == 8 ? 0x5fe6eb50c7b537a9 : 0x5f3759df) - (i >> 1);
		y = *(T*)&i;
		for (size_t i = 0; i <= iterations; i++)
			y = y * (1.5 - (x2 * y * y));
		return y;
	}

}
#endif