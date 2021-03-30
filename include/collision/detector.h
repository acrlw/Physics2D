#pragma once
#include "include/collision/algorithm/gjk.h"
#include "include/collision/algorithm/sat.h"
#include "include/collision/contact.h"
#include "include/math/math.h"
#include "include/dynamics/shape.h"
#include "include/dynamics/rigidbody.h"

namespace Physics2D
{
	class Detector
	{
	public:
		static ContactInfo detect(RigidBody* body_A, RigidBody* body_B)
		{
			ContactInfo result;
			
			if (body_A == nullptr || body_B == nullptr)
				return result;

			
		}
	private:
		
	};
}