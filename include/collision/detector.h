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
            static ContactInfo detect(Body* bodyA, Body* bodyB)
            {
                ContactInfo result;

                if (bodyA == nullptr || bodyB == nullptr)
                    return result;

                ShapePrimitive shapeA, shapeB;
                shapeA.shape = bodyA->shape();
                shapeA.rotation = bodyA->angle();
                shapeA.transform = bodyA->position();

                shapeB.shape = bodyB->shape();
                shapeB.rotation = bodyB->angle();
                shapeB.transform = bodyB->position();
                auto [isCollide, simplex] = GJK::gjk(shapeA, shapeB);
                result.isCollide = isCollide;
            	if(isCollide)
            	{
                    simplex = GJK::epa(shapeA, shapeB, simplex);
                    result = GJK::dumpInfo(shapeA, shapeB, simplex);
            	}
                return result;

            }
        private:

    };
}
#endif	
