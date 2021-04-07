#ifndef PHYSICS2D_DETECTOR_H
#define PHYSICS2D_DETECTOR_H
#include "include/collision/algorithm/gjk.h"
#include "include/collision/algorithm/sat.h"
#include "include/collision/contact.h"
#include "include/math/math.h"
#include "include/dynamics/shape.h"
#include "include/dynamics/body.h"

namespace Physics2D
{
    class Detector
    {
        public:
            static ContactInfo detect(Body* body_A, Body* body_B)
            {
                ContactInfo result;

                if (body_A == nullptr || body_B == nullptr)
                    return result;


            }
        private:

    };
}
#endif	
