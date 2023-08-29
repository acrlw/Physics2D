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
#include <iostream>



#if defined(_WIN32)
#   define PHYSICS2D_API         __declspec(dllexport)
#elif defined(__GNUC__) && ((__GNUC__ >= 4) || (__GNUC__ == 3 && __GNUC_MINOR__ >= 3))
#   define PHYSICS2D_API         __attribute__((visibility("default")))
#else
#   define PHYSICS2D_API
#endif

#define SINGLE_PRECISION

namespace Physics2D
{
	using Index = uint32_t;

	namespace Container
	{
		template <class T>
		using Vector = std::vector<T>;

		template <typename K, typename V>
		using Map = std::map<K, V>;
	}
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
		constexpr real HalfPi = Pi / 2.0f;
		constexpr real DoublePi = Pi * 2.0f;
		constexpr real ReciprocalOfPi = 0.3183098861f;
		constexpr real GeometryEpsilon = 1e-6f;
		constexpr real TrignometryEpsilon = 1e-3f;
		constexpr real CCDMinVelocity = 100.0f;
		constexpr real MaxVelocity = 1000.0f;
		constexpr real MaxAngularVelocity = 1000.0f;
		constexpr real AABBExpansionFactor = 0.0f;
		constexpr real MinLinearVelocity = 1e-4f;
		constexpr real MinAngularVelocity = 1e-4f;
		constexpr real MinEnergy = 9e-10f;
		constexpr size_t SleepCountdown = 32;
		constexpr int GJKRetryTimes = 4;
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
		constexpr real AABBExpansionFactor = 0.0;
		constexpr real MinLinearVelocity = 1e-4;
		constexpr real MinAngularVelocity = 1e-4;
		constexpr size_t SleepCountdown = 32;

	}
#endif
}

#endif
