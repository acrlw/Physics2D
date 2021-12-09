#ifndef PHYSICS2D_SYSTEM_H
#define PHYSICS2D_SYSTEM_H
#include "./body.h"
#include "./world.h"
#include "../collision/detector.h"
#include "../collision/broadphase/tree.h"
#include "../collision/continuous/ccd.h"
namespace Physics2D
{
    class PhysicsSystem
    {
    public:
        void step(const real& dt);
        PhysicsWorld& world();
        ContactMaintainer& maintainer();
        Tree& tree();
        int& positionIteration();
        int& velocityIteration();


    private:
        void updateTree();
        void solve(const real& dt);
        bool solveCCD(const real& dt);
        int m_positionIteration = 8;
        int m_velocityIteration = 6;

        PhysicsWorld m_world;
        ContactMaintainer m_maintainer;
        Tree m_tree;

    };

}
#endif
