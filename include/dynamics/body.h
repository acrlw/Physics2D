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

            Vector2 position() const
            {
                return m_position;
            }

            void setPosition(const Vector2 &position)
            {
                m_position = position;
            }

            Vector2 velocity() const
            {
                return m_velocity;
            }

            void setVelocity(const Vector2 &velocity)
            {
                m_velocity = velocity;
            }

            real angle() const
            {
                return m_angle;
            }

            void setAngle(const real &angle)
            {
                m_angle = angle;
            }

            real angularVelocity() const
            {
                return m_angularVelocity;
            }

            void setAngularVelocity(const real &angularVelocity)
            {
                m_angularVelocity = angularVelocity;
            }

            Vector2 forces() const
            {
                return m_forces;
            }

            void setForces(const Vector2 &forces)
            {
                m_forces = forces;
            }

            real torques() const
            {
                return m_torques;
            }

            void setTorques(const real &torques)
            {
                m_torques = torques;
            }

            Shape *shape() const
            {
                return m_shape;
            }

            void setShape(Shape *shape)
            {
                m_shape = shape;
                calcInertia();
            }

            BodyType type() const
            {
                return m_type;
            }

            void setType(const BodyType &type)
            {
                m_type = type;
            }

            real mass() const
            {
                return m_mass;
            }

            void setMass(const real &mass)
            {
                m_mass = mass;
            }

            real inertia() const
            {
                return m_inertia;
            }

            AABB aabb()const
            {
                ShapePrimitive primitive;
                primitive.transform = m_position;
                primitive.rotation = m_angle;
            	primitive.shape = m_shape;
                return AABB::fromShape(primitive);
            }


            real damping() const
            {
                return m_damping;
            }

            void setDamping(const real &damping)
            {
                m_damping = damping;
            }

            bool sleep() const
            {
                return m_sleep;
            }

            void setSleep(bool sleep)
            {
                m_sleep = sleep;
            }





        private:
            void calcInertia();

            real m_mass = 1;
            real m_inertia = 0;
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
