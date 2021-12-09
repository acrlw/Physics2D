#include "../../include/dynamics/system.h"
namespace Physics2D
{
    int& PhysicsSystem::positionIteration()
    {
        return m_positionIteration;
    }

    int& PhysicsSystem::velocityIteration()
    {
        return m_velocityIteration;
    }
    PhysicsWorld &PhysicsSystem::world()
    {
        return m_world;
    }

    ContactMaintainer &PhysicsSystem::maintainer()
    {
        return m_maintainer;
    }

    Tree &PhysicsSystem::tree()
    {
        return m_tree;
    }

    void PhysicsSystem::step(const real &dt)
    {
        updateTree();
        //solve ccd first, then solve normal case.
        if(!solveCCD(dt))
            solve(dt);
    }
    void PhysicsSystem::updateTree()
    {
        for (auto& elem : m_world.bodyList())
            m_tree.update(elem.get());
    }
    bool PhysicsSystem::solveCCD(const real& dt)
    {
        std::vector<Body*> bullets;
        for (auto& body : m_world.bodyList())
            if (body->type() == Body::BodyType::Bullet)
                bullets.emplace_back(body.get());

        for (auto& bullet : bullets)
        {
            //check bullet velocity threshold
            if (bullet->velocity().lengthSquare() < Constant::CCDMinVelocity && bullet->angularVelocity() < Constant::CCDMinVelocity)
                continue;
            auto potentials = CCD::query(m_tree, bullet, dt);
            if (potentials.has_value())
            {
                auto finals = CCD::earliestTOI(potentials.value());
                if (finals.has_value())
                {
                    //if toi still exist, just keep solving them until the sum of toi is greater than dt
                    real toi = finals.value();
                    solve(toi);
                    real ddt = (dt - toi) / real(Constant::CCDMaxIterations);
                    for (int i = 0; i < Constant::CCDMaxIterations; i++) {
                        updateTree();
                        solve(ddt);
                    }
                    //return solved
                    return true;
                }
            }
        }
        //there isn't a ccd case, solve nothing.
        return false;
    }
    void PhysicsSystem::solve(const real& dt)
    {
        m_world.stepVelocity(dt);

        auto potentialList = m_tree.generate();
        for (auto pair : potentialList)
        {
            auto result = Detector::detect(pair.first, pair.second);
            if (result.isColliding)
                m_maintainer.add(result);
        }
        m_maintainer.clearInactivePoints();
        m_world.prepareVelocityConstraint(dt);

        for (int i = 0; i < m_velocityIteration; ++i)
        {
            m_world.solveVelocityConstraint(dt);
            m_maintainer.solveVelocity(dt);
        }

        m_world.stepPosition(dt);

        for (int i = 0; i < m_positionIteration; ++i)
        {
            m_maintainer.solvePosition(dt);
            m_world.solvePositionConstraint(dt);
        }
        m_maintainer.deactivateAllPoints();
    }
}
