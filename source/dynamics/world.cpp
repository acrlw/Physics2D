#include "include/dynamics/world.h"

namespace Physics2D {

    Vector2 World::screenToWorld(const Vector2 &pos) const
    {
        return screenToWorld(m_leftTop, m_rightBottom, pos);
    }

    Vector2 World::worldToScreen(const Vector2 &pos) const
    {
        return worldToScreen(m_leftTop, m_rightBottom, pos);
    }

    void World::step(const number &dt)
    {

    }

    Vector2 World::worldToScreen(const Vector2 &leftTop, const Vector2 &rightBottom, const Vector2 &pos)
    {
        number origin_y = rightBottom.y;
        number origin_x = (leftTop.x + rightBottom.x) / 2;
        return Vector2(origin_x + pos.x, origin_y - pos.y);
    }

    Vector2 World::screenToWorld(const Vector2 &leftTop, const Vector2 &rightBottom, const Vector2 &pos)
    {
        number origin_y = rightBottom.y;
        number origin_x = (leftTop.x + rightBottom.x) / 2;
        return pos - Vector2(origin_x, origin_y);
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

    number World::angularVelocityDamping() const
    {
        return m_angularVelocityDamping;
    }

    void World::setAngularVelocityDamping(const number &angularVelocityDamping)
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

    number World::angularVelocityThreshold() const
    {
        return m_angularVelocityThreshold;
    }

    void World::setAngularVelocityThreshold(const number &angularVelocityThreshold)
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

    }

    void World::removeBody(Body *body)
    {

    }

    number World::width()
    {
        return m_rightBottom.x - m_leftTop.x;
    }

    number World::height()
    {
        return m_rightBottom.y - m_leftTop.y;
    }

}
