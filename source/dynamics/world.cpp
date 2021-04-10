#include "include/dynamics/world.h"

namespace Physics2D {
    World::~World()
    {
    	for(Body * body: m_bodyList)
            delete body;
    }
    Vector2 World::screenToWorld(const Vector2 &pos) const
    {
        return screenToWorld(m_leftTop, m_rightBottom, pos);
    }

    Vector2 World::worldToScreen(const Vector2 &pos) const
    {
        return worldToScreen(m_leftTop, m_rightBottom, pos);
    }

    void World::step(const real &dt)
    {

    }

    void World::setGeometry(const Vector2& leftTop, const Vector2& rightBottom)
    {
        m_leftTop = leftTop;
        m_rightBottom = rightBottom;
    }

    Vector2 World::worldToScreen(const Vector2 &leftTop, const Vector2 &rightBottom, const Vector2 &pos)
    {
        const real origin_y = rightBottom.y;
        const real origin_x = (leftTop.x + rightBottom.x) / 2;
        return Vector2(origin_x + pos.x, origin_y - pos.y);
    }

    Vector2 World::screenToWorld(const Vector2 &leftTop, const Vector2 &rightBottom, const Vector2 &pos)
    {
        const real origin_y = rightBottom.y;
        const real origin_x = (leftTop.x + rightBottom.x) / 2;
        Vector2 result = pos - Vector2(origin_x, origin_y);
        result.y = -result.y;
        return result;
    }

    std::vector<Body*> World::bodyList() const
    {
        return m_bodyList;
    }
    
    real World::bias() const
    {
        return m_bias;
    }
    
    void World::setBias(const real &bias)
    {
        m_bias = bias;
    }
    
    real World::velocityIteration() const
    {
        return m_velocityIteration;
    }
    
    void World::setVelocityIteration(const real &velocityIteration)
    {
        m_velocityIteration = velocityIteration;
    }
    
    real World::positionIteration() const
    {
        return m_positionIteration;
    }
    
    void World::setPositionIteration(const real &positionIteration)
    {
        m_positionIteration = positionIteration;
    }
    
    Vector2 World::leftTop() const
    {
        return m_leftTop;
    }
    
    void World::setLeftTop(const Vector2 &leftTop)
    {
        m_leftTop = leftTop;
    }
    
    Vector2 World::rightBottom() const
    {
        return m_rightBottom;
    }

    void World::setRightBottom(const Vector2 &rightBottom)
    {
        m_rightBottom = rightBottom;
    }

    Vector2 World::gravity() const
    {
        return m_gravity;
    }

    void World::setGravity(const Vector2 &gravity)
    {
        m_gravity = gravity;
    }

    Vector2 World::linearVelocityDamping() const
    {
        return m_linearVelocityDamping;
    }

    void World::setLinearVelocityDamping(const Vector2 &linearVelocityDamping)
    {
        m_linearVelocityDamping = linearVelocityDamping;
    }

    real World::angularVelocityDamping() const
    {
        return m_angularVelocityDamping;
    }

    void World::setAngularVelocityDamping(const real &angularVelocityDamping)
    {
        m_angularVelocityDamping = angularVelocityDamping;
    }

    Vector2 World::linearVelocityThreshold() const
    {
        return m_linearVelocityThreshold;
    }

    void World::setLinearVelocityThreshold(const Vector2 &linearVelocityThreshold)
    {
        m_linearVelocityThreshold = linearVelocityThreshold;
    }

    real World::angularVelocityThreshold() const
    {
        return m_angularVelocityThreshold;
    }

    void World::setAngularVelocityThreshold(const real &angularVelocityThreshold)
    {
        m_angularVelocityThreshold = angularVelocityThreshold;
    }

    bool World::enableGravity() const
    {
        return m_enableGravity;
    }

    void World::setEnableGravity(bool enableGravity)
    {
        m_enableGravity = enableGravity;
    }

    void World::addBody(Body *body)
    {
        m_bodyList.emplace_back(body);
    }

    void World::removeBody(Body *body)
    {
        m_bodyList.erase(std::remove(m_bodyList.begin(), m_bodyList.end(), body),
            m_bodyList.end());
        delete body;
    }

    real World::width()
    {
        return m_rightBottom.x - m_leftTop.x;
    }

    real World::height()
    {
        return m_rightBottom.y - m_leftTop.y;
    }

}
