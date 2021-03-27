#pragma once
#include "include/math/math.h"
namespace Physics2D
{
	struct ContactInfo
	{
		Vector2 contactA;
		Vector2 contactB;
		bool isCollide = false;
		Vector2 penetrationVector;
	};
}