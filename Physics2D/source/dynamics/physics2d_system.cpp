#include "physics2d_system.h"
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
    bool& PhysicsSystem::sliceDeltaTime()
    {
        return m_sliceDeltaTime;
    }

    bool& PhysicsSystem::solveJointVelocity()
    {
        return m_solveJointVelocity;
    }

    bool& PhysicsSystem::solveJointPosition()
    {
        return m_solveJointPosition;
    }

    bool& PhysicsSystem::solveContactVelocity()
    {
        return m_solveContactVelocity;
    }

    bool& PhysicsSystem::solveContactPosition()
    {
        return m_solveContactPosition;
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

    UniformGrid& PhysicsSystem::grid()
    {
        return m_grid;
    }

    void PhysicsSystem::step(const real &dt)
    {
        //updateTree();
        //updateGrid();
        

        //solve ccd first, then solve normal case.
        if(!solveCCD(dt))
            solve(dt);

        updateTree();
    }
    void PhysicsSystem::updateTree()
    {
        //bvh
        for (const auto& elem : m_world.bodyList())
            m_tree.update(elem.get());
    }

    void PhysicsSystem::updateGrid()
    {
        m_grid.updateAll();
    }

    bool PhysicsSystem::solveCCD(const real& dt)
    {
        Container::Vector<Body*> bullets;
        for (const auto& body : m_world.bodyList())
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
                    updateTree();
                    solve(toi);
                    real ddt = (dt - toi) / real(Constant::CCDMaxIterations);
                    for (int i = 0; i < Constant::CCDMaxIterations; ++i) {
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

        //Sweep And Prune

        //Container::Vector<Body*> bodies;
        //bodies.reserve(m_world.bodyList().size());
        //for(auto&& elem: m_world.bodyList())
        //    bodies.emplace_back(elem.get());
        //
        //auto potentialList = SweepAndPrune::generate(bodies);


        //BVH
        real vdt = dt;
        real pdt = dt;
        if(m_sliceDeltaTime)
        {
        	vdt = dt / real(m_velocityIteration);
        	pdt = dt / real(m_positionIteration);
        }

        m_world.stepVelocity(dt);
        //auto potentialList = m_grid.generate();

        auto potentialList = m_tree.generate();
        for (auto pair : potentialList)
        {
            auto result = Detector::detect(pair.first, pair.second);
            if (result.isColliding) {
                m_maintainer.add(result);
            }
        }
        m_maintainer.clearInactivePoints();

    	m_world.prepareVelocityConstraint(dt);

        for (int i = 0; i < m_velocityIteration; ++i)
        {
            if(m_solveJointVelocity)
				m_world.solveVelocityConstraint(vdt);

            if (m_solveContactVelocity)
				m_maintainer.solveVelocity(vdt);
        }
        m_world.stepPosition(dt);

        //solve penetration use contact pairs from previous velocity solver settings
        //TODO: Can generate another contact table just for position solving
        for (int i = 0; i < m_positionIteration; ++i)
        {
            if(m_solveContactPosition)
				m_maintainer.solvePosition(pdt);

            if(m_solveJointPosition)
				m_world.solvePositionConstraint(pdt);
        }

        m_maintainer.deactivateAllPoints();
    }
}
