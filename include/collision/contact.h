#ifndef PHYSICS2D_CONTACT_H
#define PHYSICS2D_CONTACT_H
#include "include/math/linear/linear.h"
namespace Physics2D
{
    struct ContactInfo
    {
            Vector2 contactA;
            Vector2 contactB;
            bool isCollide = false;
            Vector2 penetration;
    };
}
#endif
