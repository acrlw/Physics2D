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
            Body() = default;
            Vector2& position();

            Vector2& velocity();

            real& angle();

            real& angularVelocity();

            Vector2& forces();
            void clearTorque();

            real& torques();

            std::shared_ptr<Shape> shape() const;
            void setShape(std::shared_ptr<Shape> shape);

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

            void applyImpulse(const Vector2& impulse, const Vector2& r);
            Vector2 toLocalPoint(const Vector2& point)const;
            Vector2 toWorldPoint(const Vector2& point)const;
            Vector2 toActualPoint(const Vector2& point)const;
        private:
            void calcInertia();

            real m_mass = 0;
            real m_inertia = 0;
            real m_invMass = 0;
            real m_invInertia = 0;
    	
            Vector2 m_position;
            Vector2 m_velocity;
            real m_angle = 0;
            real m_angularVelocity = 0;
    	
            Vector2 m_forces;
            real m_torques = 0;
    	
            std::shared_ptr<Shape>m_shape;
            BodyType m_type = BodyType::Static;
    	
            bool m_sleep = false;
            real m_damping = 0.7;

            BodyState m_bodyState;
    };
}
#endif
