#ifndef ALWORLD_H
#define ALWORLD_H
#include <vector>
#include <alsettings.h>
#include <albody.h>
#include <alcollision.h>
class alWorld
{
    enum WorldType{
        Vertical,
        Horizontal
    };

public:
    alWorld(){
        m_gravity = alVector2(0.0f, 9.8f);
        m_airFrictionCoefficient = alAirFrictionCoefficient;
        m_planeFrictionCoefficient = alFrictionCoefficient;
    }

    std::vector<alBody *>& bodyList()
    {
        return m_bodyList;
    }

    void setBodyList(const std::vector<alBody *> &bodyList)
    {
        m_bodyList = bodyList;
    }

    void step(const float& dt = 1.0f / 60.0f, const float& dt_last = 1.0f / 60.0f)
    {
        foreach(auto body, m_bodyList)
        {
            if (!body->sleep())
            {
                //semi-euler integrator
                if(body->velocity().lengthSquare() > 0.1f){
                    alVector2 dir = body->velocity().getNormalizedVector();
                    body->addForce(dir.negate() * 0.6f);
                }
                if(body->angularVelocity() > 0.1f){
                    float dir = -1 * body->angularVelocity();
                    body->angularAcceleration() += dir * 0.1f;
                }
                body->velocity() +=  dt * (m_gravity + body->invMass() * body->forces());
                body->angularVelocity() +=  dt * body->invMass() * body->torque();

                body->position() +=  body->velocity() * dt;
                body->angle() += body->angularVelocity() * dt;

                body->setAcceleration(alVector2());
                body->setAngularAcceleration(0.0f);
                body->setTorque(0.0f);
                body->setForces(alVector2());

                //verlet
//                float dt2 = pow(dt, 2);
//                auto a = m_gravity + body->invMass() * body->forces();
//                auto b = body->invMass() * body->torque();
//                alVector2 dx = body->position() - body->lastPosition();
//                body->velocity() = dx / dt_last + a * dt;
//                body->lastPosition() = body->position();
//                body->position() = body->position() + dx + a * dt2;

//                body->angularVelocity() = ((body->angle() - body->lastAngle()) / dt_last);
//                body->lastAngle() = body->angle();
//                body->angle() += body->angularVelocity() * dt + b * dt2;

//                body->setAcceleration(alVector2());
//                body->setAngularAcceleration(0.0f);
//                body->setTorque(0.0f);
//                body->setForces(alVector2());
            }

        }

        collide();
    }

    alVector2 gravity() const
    {
        return m_gravity;
    }

    void setGravity(const alVector2 &gravity)
    {
        m_gravity = gravity;
    }

    float airFrictionCoefficient() const
    {
        return m_airFrictionCoefficient;
    }

    void setAirFrictionCoefficient(float airFrictionCoefficient)
    {
        m_airFrictionCoefficient = airFrictionCoefficient;
    }

    float planeFrictionCoefficient() const
    {
        return m_planeFrictionCoefficient;
    }

    void setPlaneFrictionCoefficient(float planeFrictionCoefficient)
    {
        m_planeFrictionCoefficient = planeFrictionCoefficient;
    }


private:
    std::vector<alBody *> m_bodyList;
    alVector2 m_gravity;
    float m_airFrictionCoefficient;
    float m_planeFrictionCoefficient;

    void collide()
    {
        alGJKCollisionDetector gjk;
        ContactInfo result;
        for(size_t i = 0;i < m_bodyList.size();i++)
        {
            for(size_t j = 1;j < m_bodyList.size();j++)
            {
                float l1 = m_bodyList[i]->aabbLength();
                float l2 = m_bodyList[j]->aabbLength();
//                if(m_bodyList[i]->type() == BodyType::Polygon)
//                    qDebug() << "polygon aabb: " << l1;
//                if(m_bodyList[j]->type() == BodyType::Polygon)
//                    qDebug() << "polygon aabb: " << l2;
                alVector2 posDiff = m_bodyList[i]->position() - m_bodyList[j]->position();
                if(posDiff.lengthSquare() <= pow(l1 + l2, 2))
                {
                    result = gjk.detect(nullptr, m_bodyList[i], m_bodyList[j]);
                    m_bodyList[i]->setIsTouched(result.getIsCollide());
                    m_bodyList[j]->setIsTouched(result.getIsCollide());
                    if(result.getIsCollide())
                    {
                        alVector2 penetr = result.getPenetrationVector();

                        m_bodyList[i]->applyImpulse(penetr * 0.5, result.getBody1ContactPoint());
                        m_bodyList[i]->setForces(penetr * 100);
                        //m_bodyList[i]->velocity() += penetr * 5;
                        m_bodyList[j]->applyImpulse(penetr.negate() * 0.5, result.getBody2ContactPoint());
                        m_bodyList[j]->setForces(penetr * 100);
                        //m_bodyList[j]->velocity() += penetr * 5;
//                        float m1 = m_bodyList[i]->mass();
//                        float m2 = m_bodyList[j]->mass();
//                        alVector2 v1 = m_bodyList[i]->velocity();
//                        alVector2 v2 = m_bodyList[j]->velocity();
//                        std::cout << "after collision, v1: " << (v1 * (m1 - m2) / (m1 + m2)) + (v2 * (2 * m2) * (m1 + m2)) << std::endl;
//                        std::cout << "after collision, v2: " << (v1 * (2 * m1) / (m1 + m2)) + (v2 * (m2 - m1) * (m1 + m2)) << std::endl;
                        //m_bodyList[i]->position() += result.getPenetrationVector();
                    }
                }
            }
        }
    }
    void broadphase()
    {

    }
};

#endif // ALWORLD_H
