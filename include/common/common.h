#ifndef COMMON_H
#define COMMON_H

#include "cassert"
#include <cmath>
#include <cfloat>
#include <vector>
#include <tuple>
#include <optional>
namespace Physics2D
{
#ifdef SINGLE_PRECISION
	using number = float;
	const float epsilon = FLT_EPSILON;
	const float  PI = 3.14159265358979f;

#else
	using number = double;
	const float EPSILON = DBL_EPSILON;
    const double PI = 3.141592653589793238463;
#endif
	const unsigned int GJKIteration = 50;
	const number EPAEPSILON = 0.0001;

    template <typename T, size_t iterations = 2> inline T fastInverseSqrt(T x) {
        typedef typename std::conditional<sizeof(T) == 8, std::int64_t, std::int32_t>::type Tint;
        T y = x;
        T x2 = y * 0.5;
        Tint i = *(Tint*)&y;
        i = (sizeof(T) == 8 ? 0x5fe6eb50c7b537a9 : 0x5f3759df) - (i >> 1);
        y = *(T*)&i;
    	for(size_t i = 0;i <= iterations;i++)
           y = y * (1.5 - (x2 * y * y));
        return y;
    }

}

#endif