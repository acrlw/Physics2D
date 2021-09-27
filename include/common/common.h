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
#include <memory>
#include <map>

namespace Physics2D
{
#ifdef SINGLE_PRECISION
	using real = float;
	namespace Constant
	{
		constexpr real Epsilon = FLT_EPSILON;
		constexpr real Max = FLT_MAX;
		constexpr real PositiveMin = FLT_MIN;
		constexpr real NegativeMin = -Max;
		constexpr real Pi = 3.141592653589793238463;
		constexpr real HalfPi = Constant::Pi / 2;
		constexpr real DoublePi = Constant::Pi * 2;
		constexpr real ReciprocalOfPi = 0.3183098861837907f;
		constexpr real GeometryEpsilon = 0.0000001;
		constexpr real MaxVelocity = 1000.0;
		constexpr real MaxAngularVelocity = 1000.0;
	}
#else
	using real = double;
	namespace Constant
	{
		constexpr real Epsilon = DBL_EPSILON;
		constexpr real Max = DBL_MAX;
		constexpr real PositiveMin = DBL_MIN;
		constexpr real NegativeMin = -Max;
		constexpr real Pi = 3.141592653589793238463;
		constexpr real HalfPi = Constant::Pi / 2;
		constexpr real DoublePi = Constant::Pi * 2;
		constexpr real ReciprocalOfPi = 0.3183098861837907;
		constexpr real GeometryEpsilon = 0.0000001;
		constexpr real MaxVelocity = 1000.0;
		constexpr real MaxAngularVelocity = 1000.0;

	}
#endif
}

#endif
