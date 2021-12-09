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
#define SINGLE_PRECISION
namespace Physics2D
{
#ifdef SINGLE_PRECISION
	using real = float;
	namespace Constant
	{
		constexpr unsigned int SimplexMax = 8;
		constexpr unsigned int CCDMaxIterations = 15;
		constexpr real Epsilon = FLT_EPSILON;
		constexpr real Max = FLT_MAX;
		constexpr real PositiveMin = FLT_MIN;
		constexpr real NegativeMin = -Max;
		constexpr real Pi = 3.14159265f;
		constexpr real HalfPi = Constant::Pi / 2.0f;
		constexpr real DoublePi = Constant::Pi * 2.0f;
		constexpr real ReciprocalOfPi = 0.3183098861f;
		constexpr real GeometryEpsilon = 0.00001f;
		constexpr real CCDMinVelocity = 100.0f;
		constexpr real MaxVelocity = 1000.0f;
		constexpr real MaxAngularVelocity = 1000.0f;
	}
#else
	using real = double;
	namespace Constant
	{
		constexpr unsigned int SimplexMax = 8;
		constexpr real Epsilon = DBL_EPSILON;
		constexpr real Max = DBL_MAX;
		constexpr real PositiveMin = DBL_MIN;
		constexpr real NegativeMin = -Max;
		constexpr real Pi = 3.141592653589793238463;
		constexpr real HalfPi = Constant::Pi / 2.0;
		constexpr real DoublePi = Constant::Pi * 2.0;
		constexpr real ReciprocalOfPi = 0.3183098861837907;
		constexpr real GeometryEpsilon = 0.0000001;
		constexpr real CCDMinVelocity = 100.0;
		constexpr real MaxVelocity = 1000.0;
		constexpr real MaxAngularVelocity = 1000.0;

	}
#endif
}

#endif
