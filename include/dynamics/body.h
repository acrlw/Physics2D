#ifndef PHYSICS2D_BODY_H
#define PHYSICS2D_BODY_H
#include "include/collision/broadphase/aabb.h"
#include "include/math/math.h"
#include "include/common/common.h"
#include "include/dynamics/shape.h"
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

            Vector2 position() const;
            void setPosition(const Vector2 &position);

            Vector2 velocity() const;
            void setVelocity(const Vector2 &velocity);

            real angle() const;
            void setAngle(const real &angle);

            real angularVelocity() const;
            void setAngularVelocity(const real &angularVelocity);

            Vector2 forces() const;
            void setForces(const Vector2 &forces);

            real torques() const;
            void setTorques(const real &torques);

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

        private:
            void calcInertia();

            real m_mass = 1;
            real m_inertia = 0;
            real m_invMass = 1;
            real m_invInertia = 0;
    	
            Vector2 m_position;
            Vector2 m_velocity;
            real m_angle = 0;
            real m_angularVelocity = 0;
    	
            Vector2 m_forces;
            real m_torques = 0;
    	
            Shape *m_shape = nullptr;
            BodyType m_type = BodyType::Static;
    	
            bool m_sleep = true;
            real m_damping = 0.5f;
    	
    };
}
#endif
