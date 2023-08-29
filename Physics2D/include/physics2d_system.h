#ifndef PHYSICS2D_SYSTEM_H
#define PHYSICS2D_SYSTEM_H
#include "physics2d_body.h"
#include "physics2d_world.h"
#include "physics2d_detector.h"
#include "physics2d_tree.h"
#include "physics2d_ccd.h"
#include "physics2d_sap.h"
#include "physics2d_grid.h"

namespace Physics2D
{
	class PHYSICS2D_API PhysicsSystem
	{
	public:
		void step(const real& dt);
		PhysicsWorld& world();
		ContactMaintainer& maintainer();
		Tree& tree();
		UniformGrid& grid();
		int& positionIteration();
		int& velocityIteration();
		bool& sliceDeltaTime();
		bool& solveJointVelocity();
		bool& solveJointPosition();
		bool& solveContactVelocity();
		bool& solveContactPosition();

	private:
		void updateTree();
		void updateGrid();
		void solve(const real& dt);
		bool solveCCD(const real& dt);
		int m_positionIteration = 3;
		int m_velocityIteration = 8;
		bool m_sliceDeltaTime = false;
		bool m_solveJointVelocity = true;
		bool m_solveJointPosition = true;
		bool m_solveContactVelocity = true;
		bool m_solveContactPosition = true;
		PhysicsWorld m_world;
		ContactMaintainer m_maintainer;
		Tree m_tree;
		UniformGrid m_grid;
	};
}
#endif
