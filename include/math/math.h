#ifndef PHYSICS2D_MATH_H
#define PHYSICS2D_MATH_H
#include "include/common/common.h"

namespace Physics2D
{
	//trigonometric function
	inline number sinx(const number& x)
	{
		return std::sin(x);
	}
	inline number cosx(const number& x)
	{
		return std::cos(x);
	}
	inline number tanx(const number& x)
	{
		return std::tan(x);
	}
	inline number arcsinx(const number& x)
	{
		return std::asin(x);
	}
	inline number arccosx(const number& x)
	{
		return std::acos(x);
	}
	inline number arctanx(const number& x)
	{
		return std::atan(x);
	}
	inline number max(const number& a, const number& b)
	{
		return std::max(a, b);
	}
	inline number min(const number& a, const number& b)
	{
		return std::min(a, b);
	}
	inline number abs_max(const number& a, const number& b)
	{
		return std::max(std::fabs(a), std::fabs(b));
	}
	inline number abs_min(const number& a, const number& b)
	{
		return std::min(std::fabs(a), std::fabs(b));
	}
	//other
	inline int sign(const number& num)
	{
		return num > 0 ? 1 : -1;
	}
	inline number clamp(const number& num, const number& low, const number& high)
	{
		return std::clamp(num, low, high);
	}
	//basic number utility
	inline void numberSwap(number& lhs, number& rhs)
	{
		const number temp = lhs;
		lhs = rhs;
		rhs = temp;
	}
	inline bool numberEqual(const number& lhs, const number& rhs)
	{
		return abs(lhs - rhs) < EPSILON;
	}
	inline number angleToRadian(const number& angle)
	{
		return angle * (180.0f / PI);
	}
	inline number radianToAngle(const number& radian)
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