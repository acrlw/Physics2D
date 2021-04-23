#ifndef PHYSICS2D_DETECTOR_H
#define PHYSICS2D_DETECTOR_H
#include "include/collision/algorithm/gjk.h"
#include "include/collision/algorithm/sat.h"
#include "include/collision/contact.h"
#include "include/math/math.h"
#include "include/geometry/shape.h"
#include "include/dynamics/body.h"

namespace Physics2D
{
    class Detector
    {
        public:
            static CollisionInfo detect(Body* bodyA, Body* bodyB)
            {
                CollisionInfo result;

                if (bodyA == nullptr || bodyB == nullptr)
                    return result;

                if (bodyA == bodyB)
                    return result;

                result.bodyA = bodyA;
                result.bodyB = bodyB;
            	
                ShapePrimitive shapeA, shapeB;
                shapeA.shape = bodyA->shape();
                shapeA.rotation = bodyA->angle();
                shapeA.transform = bodyA->position();

                shapeB.shape = bodyB->shape();
                shapeB.rotation = bodyB->angle();
                shapeB.transform = bodyB->position();

                AABB a = AABB::fromShape(shapeA);
                AABB b = AABB::fromShape(shapeB);
                if (!a.collide(b))
                    return result;
            	
                auto [isColliding, simplex] = GJK::gjk(shapeA, shapeB);
            	if(isColliding)
            	{
                    simplex = GJK::epa(shapeA, shapeB, simplex);
                    result.info = GJK::dumpInfo(shapeA, shapeB, simplex);
            	}
                
                return result;

            }
			static CollisionInfo distance(Body* bodyA, Body* bodyB)
            {

                CollisionInfo result;

                if (bodyA == nullptr || bodyB == nullptr)
                    return result;

                if (bodyA == bodyB)
                    return result;

                result.bodyA = bodyA;
                result.bodyB = bodyB;

                ShapePrimitive shapeA, shapeB;
                shapeA.shape = bodyA->shape();
                shapeA.rotation = bodyA->angle();
                shapeA.transform = bodyA->position();

                shapeB.shape = bodyB->shape();
                shapeB.rotation = bodyB->angle();
                shapeB.transform = bodyB->position();
                
                result.info = GJK::distance(shapeA, shapeB);

                return result;

            }
        private:

    };
}
#endif	
