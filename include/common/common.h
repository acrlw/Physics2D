#pragma once
#include "cassert"
#include <cmath>
#include <cfloat>
#include <vector>
namespace Physics2D
{
#ifdef SINGLE_PRECISION
	typedef float number;
	const float epsilon = FLT_EPSILON;
#else
	typedef double number;
	const float EPSILON = DBL_EPSILON;
#endif
	
}
