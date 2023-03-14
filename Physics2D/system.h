#ifndef PHYSICS2D_SYSTEM_H
#define PHYSICS2D_SYSTEM_H
#include "body.h"
#include "world.h"
#include "detector.h"
#include "tree.h"
#include "ccd.h"
#include "sap.h"
#include "grid.h"
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
        int m_positionIteration = 6;
        int m_velocityIteration = 8;

        PhysicsWorld m_world;
        ContactMaintainer m_maintainer;
        Tree m_tree;
        UniformGrid m_grid;
    };

}
#endif
