#include "body.h"
namespace Physics2D {

    Vector2& Body::position()
    {
        return m_position;
    }
    

    Vector2& Body::velocity() 
    {
        return m_velocity;
    }
    

    real& Body::rotation() 
    {
        return m_rotation;
    }

    real& Body::angularVelocity()
    {
        return m_angularVelocity;
    }

    Vector2& Body::forces()
    {
        return m_forces;
    }
    
	
	void Body::clearTorque()
    {
        m_torques = 0;
    }
	
    real& Body::torques()
    {
        return m_torques;
    }
    Vector2& Body::lastPosition()
    {
        return m_lastPosition;
    }
    real& Body::lastRotation()
    {
        return m_lastRotation;
    }
    uint32_t& Body::sleepCountdown()
    {
        return m_sleepCountdown;
    }
    Shape* Body::shape() const
    {
        return m_shape;
    }

    void Body::setShape(Shape* shape)
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
    	
    	if(realEqual(mass,Constant::Max))
            m_invMass = 0;
        else
			m_invMass = !realEqual(mass, 0) ? 1.0f / mass : 0;
    	
        calcInertia();
    }

    real Body::inertia() const
    {
        return m_inertia;
    }

    AABB Body::aabb(const real &factor) const
    {
        ShapePrimitive primitive;
        primitive.transform = m_position;
        primitive.rotation = m_rotation;
        primitive.shape = m_shape;
        return AABB::fromShape(primitive, factor);
    }

    real Body::friction() const
    {
        return m_friction;
    }

    void Body::setFriction(const real &friction)
    {
        m_friction = friction;
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

    Body::PhysicsAttribute Body::physicsAttribute() const
    {
        return {m_position, m_velocity, m_rotation, m_angularVelocity};
    }

    void Body::setPhysicsAttribute(const PhysicsAttribute& info)
    {
        m_position = info.position;
        m_rotation = info.rotation;
        m_velocity = info.velocity;
        m_angularVelocity = info.angularVelocity;
    }

    void Body::stepPosition(const real& dt)
    {
        m_position += m_velocity * dt;
        m_rotation += m_angularVelocity * dt;
    }

    void Body::applyImpulse(const Vector2& impulse, const Vector2& r)
    {
        m_velocity += m_invMass * impulse;
        m_angularVelocity += m_invInertia * r.cross(impulse);
    }
    Vector2 Body::toLocalPoint(const Vector2& point)const
    {
        return Matrix2x2(-m_rotation).multiply(point - m_position);
    }

    Vector2 Body::toWorldPoint(const Vector2& point) const
    {
        return Matrix2x2(m_rotation).multiply(point) + m_position;
    }
    Vector2 Body::toActualPoint(const Vector2& point) const
    {
        return Matrix2x2(m_rotation).multiply(point);
    }

    uint32_t Body::id() const
    {
        return m_id;
    }

    void Body::setId(const uint32_t& id)
    {
        m_id = id;
    }

    uint32_t Body::bitmask() const
    {
        return m_bitmask;
    }

    void Body::setBitmask(const uint32_t& bitmask)
    {
        m_bitmask = bitmask;
    }

    real Body::restitution() const
    {
        return m_restitution;
    }

    void Body::setRestitution(const real& restitution)
    {
        m_restitution = restitution;
    }

    void Body::calcInertia()
    {
        switch (m_shape->type()) {
        case Shape::Type::Circle:
        {
            const Circle* circle = static_cast<Circle*>(m_shape);

            m_inertia = m_mass * circle->radius() * circle->radius() / 2;
            break;
        }
        case Shape::Type::Polygon:
        {
            const Polygon* polygon = static_cast<Polygon*>(m_shape);

            const Vector2 center = polygon->center();
            real sum1 = 0.0;
            real sum2 = 0.0;

            for (uint32_t i = 0; i < polygon->vertices().size() - 1; i++)
            {
                Vector2 n1 = polygon->vertices()[i] - center;
                Vector2 n2 = polygon->vertices()[i + 1] - center;
                real cross = std::fabs(n1.cross(n2));
                real dot = n2.dot(n2) + n2.dot(n1) + n1.dot(n1);
                sum1 += cross * dot;
                sum2 += cross;
            }

            m_inertia = m_mass * (1.0f / 6.0f) * sum1 / sum2;
            break;
        }
        case Shape::Type::Ellipse:
        {
            const Ellipse* ellipse = static_cast<Ellipse*>(m_shape);

            const real a = ellipse->A();
            const real b = ellipse->B();
            m_inertia = m_mass * (a * a + b * b) * (1.0f / 5.0f);

            break;
        }
        case Shape::Type::Capsule:
        {
            const Capsule* capsule = static_cast<Capsule*>(m_shape);
            real r = 0, h = 0, massS = 0, inertiaS = 0, massC = 0, inertiaC = 0, volume = 0;

            if (capsule->width() >= capsule->height())//Horizontal
            {
                r = capsule->height() / 2.0f;
                h = capsule->width() - capsule->height();
            }
            else//Vertical
            {
                r = capsule->width() / 2.0f;
                h = capsule->height() - capsule->width();
            }

            volume = Constant::Pi * r * r + h * 2 * r;
            real rho = m_mass / volume;
            massS = rho * Constant::Pi * r * r;
            massC = rho * h * 2.0f * r;
            inertiaC = (1.0f / 12.0f) * massC * (h * h + (2.0f * r) * (2.0f * r));
            inertiaS = massS * r * r * 0.5f;
            m_inertia = inertiaC + inertiaS + massS * (3.0f * r + 2.0f * h) * h / 8.0f;
            break;
        }
        default:
            break;
        }
        if (realEqual(m_mass, Constant::Max))
            m_invInertia = 0;
        else
			m_invInertia = !realEqual(m_inertia, 0) ? 1.0f / m_inertia : 0;
    }

    Body::Relation::RelationID Body::Relation::generateRelationID(Body* bodyA, Body* bodyB)
    {
        assert(bodyA != nullptr && bodyB != nullptr);
        //Combine two 32-bit id into one 64-bit id in binary form
        //By Convention: bodyA.id < bodyB.id
        auto bodyAId = bodyA->id();
        auto bodyBId = bodyB->id();
        if(bodyAId > bodyBId)
            std::swap(bodyAId, bodyBId);
        
        auto pair = std::pair{ bodyAId, bodyBId };
        auto result = reinterpret_cast<uint64_t&>(pair);
        return result;
    }

    Body::Relation Body::Relation::generateRelation(Body* bodyA, Body* bodyB)
    {
        assert(bodyA != nullptr && bodyB != nullptr);
        Body::Relation result;
        auto bodyAId = bodyA->id();
        auto bodyBId = bodyB->id();
        if (bodyAId > bodyBId)
            std::swap(bodyA, bodyB);

        result.relationID = generateRelationID(bodyA, bodyB);
        result.bodyA = bodyA;
        result.bodyB = bodyB;
        return result;
    }



    void Body::PhysicsAttribute::step(const real& dt)
    {
        position += velocity * dt;
        rotation += angularVelocity * dt;
    }

}
