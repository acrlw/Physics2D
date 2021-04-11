#ifndef PHYSICS2D_COMMON_H
#define PHYSICS2D_COMMON_H

#include "cassert"
#include <cmath>
#include <cfloat>
#include <vector>
#include <tuple>
#include <optional>
#include <algorithm>
#include <functional>

namespace Physics2D
{
#ifdef SINGLE_PRECISION
	using real = float;
	namespace Constant
	{
		const real Epsilon = FLT_EPSILON;
		const real Max = FLT_MAX;
		const real Min = FLT_MIN;
		const real NegativeMin = -Max;
		const real Pi = 3.14159265358979f;
		const real ReciprocalOfPi = 0.3183098861837907f;
		const real GJKIteration = 50;
		const real EPAEpsilon = 0.0001;
	}
#else
	using real = double;
#endif
	namespace Constant
	{
		const real Epsilon = DBL_EPSILON;
		const real Max = DBL_MAX;
		const real Min = DBL_MIN;
		const real NegativeMin = -Max;
		const real Pi = 3.141592653589793238463;
		const real ReciprocalOfPi = 0.3183098861837907;
		const real GJKIteration = 50;
		const real EPAEpsilon = 0.0001;
	}
}

#endif
