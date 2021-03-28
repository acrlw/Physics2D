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


			ShapePrimitive shapeA, shapeB;
			shapeA.angle = body_A->angle();
			shapeA.shape = body_A->shape();
			
			shapeB.angle = body_B->angle();
			shapeB.shape = body_B->shape();
			shapeB.position = body_B->position() - body_A->position();

			return GJK::test(shapeA, shapeB);
		}
	private:
		
	};
}