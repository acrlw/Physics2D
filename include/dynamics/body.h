#ifndef PHYSICS2D_BODY_H
#define PHYSICS2D_BODY_H
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

            number angle() const
            {
                return m_angle;
            }

            void setAngle(const number &angle)
            {
                m_angle = angle;
            }

            number angularVelocity() const
            {
                return m_angularVelocity;
            }

            void setAngularVelocity(const number &angularVelocity)
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

            number torques() const
            {
                return m_torques;
            }

            void setTorques(const number &torques)
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

            number mass() const
            {
                return m_mass;
            }

            void setMass(const number &mass)
            {
                m_mass = mass;
            }

            number inertia() const
            {
                return m_inertia;
            }

            std::tuple<Vector2, Vector2> aabb()const
            {

            }


            number damping() const
            {
                return m_damping;
            }

            void setDamping(const number &damping)
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

            number m_mass = 1;
            number m_inertia = 0;
            Vector2 m_position;
            Vector2 m_velocity;
            number m_angle = 0;
            number m_angularVelocity = 0;
            Vector2 m_forces;
            number m_torques = 0;
            Shape *m_shape = nullptr;
            BodyType m_type = BodyType::Static;
            bool m_sleep = true;
            number m_damping = 0.5f;
    };
}
#endif
