#ifndef PHYSICS2D_BODY_H
#define PHYSICS2D_BODY_H
#include "include/collision/broadphase/aabb.h"
#include "include/math/math.h"
#include "include/common/common.h"
#include "include/geometry/shape.h"
#include "include/math/integrator.h"

namespace Physics2D
{
    class Body
    {
        public:
            enum class BodyType
                {
                Kinematic,
                Static,
                Dynamic,
                Bullet
                };
            Body();
            Vector2& position();

            Vector2& velocity();

            real& angle();

            real& angularVelocity();

            Vector2& forces();
            void clearTorque();

            real& torques();

            Shape *shape() const;
            void setShape(Shape *shape);

            BodyType type() const;
            void setType(const BodyType &type);

            real mass() const;
            void setMass(const real &mass);

            real inertia() const;

            AABB aabb(const real& factor = 1)const;

            real damping() const;
            void setDamping(const real &damping);

            bool sleep() const;
            void setSleep(bool sleep);

            real inverseMass()const;
            real inverseInertia()const;

            void applyImpulse(const Vector2& force, const Vector2& r);
            Vector2 toLocalPoint(const Vector2& point)const;
            Vector2 toWorldPoint(const Vector2& point)const;

        private:
            void calcInertia();

            real m_mass;
            real m_inertia;
            real m_invMass;
            real m_invInertia;
    	
            Vector2 m_position;
            Vector2 m_velocity;
            real m_angle;
            real m_angularVelocity;
    	
            Vector2 m_forces;
            real m_torques;
    	
            Shape *m_shape;
            BodyType m_type;
    	
            bool m_sleep;
            real m_damping;

            BodyState m_bodyState;
    };
}
#endif
