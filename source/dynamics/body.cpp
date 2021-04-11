#include "include/dynamics/body.h"

namespace Physics2D {
    Body::Body() : m_mass(1), m_inertia(0), m_invMass(1), m_invInertia(0),
	m_angle(0), m_angularVelocity(0), m_torques(0),
	m_shape(nullptr), m_type(BodyType::Static), m_sleep(true), m_damping(0.8f)
    {
    	
    }
    Vector2 Body::position() const
    {
        return m_position;
    }

    void Body::setPosition(const Vector2 &position)
    {
        m_position = position;
    }

    Vector2 Body::velocity() const
    {
        return m_velocity;
    }

    void Body::setVelocity(const Vector2 &velocity)
    {
        m_velocity = velocity;
    }

    real Body::angle() const
    {
        return m_angle;
    }

    void Body::setAngle(const real &angle)
    {
        m_angle = angle;
    }

    real Body::angularVelocity() const
    {
        return m_angularVelocity;
    }

    void Body::setAngularVelocity(const real &angularVelocity)
    {
        m_angularVelocity = angularVelocity;
    }

    Vector2 Body::forces() const
    {
        return m_forces;
    }

    void Body::setForces(const Vector2 &forces)
    {
        m_forces = forces;
    }

    real Body::torques() const
    {
        return m_torques;
    }

    void Body::setTorques(const real &torques)
    {
        m_torques = torques;
    }

    Shape *Body::shape() const
    {
        return m_shape;
    }

    void Body::setShape(Shape *shape)
    {
        m_shape = shape;
        calcInertia();
    }

    Body::BodyType Body::type() const
    {
        return m_type;
    }

    void Body::setType(const Body::BodyType &type)
    {
        m_type = type;
    }

    real Body::mass() const
    {
        return m_mass;
    }

    void Body::setMass(const real &mass)
    {
        m_mass = mass;
        m_invMass = !realEqual(mass, 0) ? 1.0f / mass : 0;
    }

    real Body::inertia() const
    {
        return m_inertia;
    }

    AABB Body::aabb(const real &factor) const
    {
        ShapePrimitive primitive;
        primitive.transform = m_position;
        primitive.rotation = m_angle;
        primitive.shape = m_shape;
        return AABB::fromShape(primitive, factor);
    }

    real Body::damping() const
    {
        return m_damping;
    }

    void Body::setDamping(const real &damping)
    {
        m_damping = damping;
    }

    bool Body::sleep() const
    {
        return m_sleep;
    }

    void Body::setSleep(bool sleep)
    {
        m_sleep = sleep;
    }

    real Body::inverseMass() const
    {
        return m_invMass;
    }

    real Body::inverseInertia() const
    {
        return m_invInertia;
    }

    void Body::calcInertia()
    {
        switch (m_shape->type()) {
            case Shape::Type::Circle:
            {
                const Circle * circle = dynamic_cast<Circle*>(m_shape);

                m_inertia = m_mass * circle->radius() * circle->radius() * (0.5f);
                break;
            }
            case Shape::Type::Polygon:
            {
                const Polygon * polygon = dynamic_cast<Polygon*>(m_shape);

                const Vector2 center = polygon->center();
                real sum1 = 0.0f;
                real sum2 = 0.0f;

                for (uint32_t i = 0; i < polygon->vertices().size() - 1; i++)
                {
                    Vector2 n1 = polygon->vertices()[i] - center;
                    Vector2 n2 = polygon->vertices()[i + 1] - center;
                    real cross = abs(n1.cross(n2));
                    real dot = n2.dot(n2) + n2.dot(n1) + n1.dot(n1);
                    sum1 += cross * dot;
                    sum2 += cross;
                }

                m_inertia = (m_mass * (1.0f / 6.0f)) * sum1 / sum2;
                break;
            }
            case Shape::Type::Ellipse:
            {
                const Ellipse * ellipse = dynamic_cast<Ellipse*>(m_shape);

                const real a = ellipse->A();
                const real b = ellipse->B();
                m_inertia = m_mass * (a*a + b*b) * (0.25f);

                break;
            }
            default:
                break;
        }
        m_invInertia = !realEqual(m_inertia, 0) ? 1.0f / m_inertia : 0;
    }

}
