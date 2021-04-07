#ifndef PHYSICS2D_COMMON_H
#define PHYSICS2D_COMMON_H

#include "cassert"
#include <cmath>
#include <cfloat>
#include <vector>
#include <tuple>
#include <optional>
#include <stack>
#include <algorithm>
#include <functional>

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


}

#endif