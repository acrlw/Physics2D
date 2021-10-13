#ifndef PHYSICS2D_SCENES_BROADPHASE_H
#define PHYSICS2D_SCENES_BROADPHASE_H
#include "testbed/frame.h"
namespace Physics2D
{
	class BroadPhaseFrame : public Frame
	{
	public:
		BroadPhaseFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Broadphase", world, maintainer, tree, dbvh)
		{

		}
		void load() override
		{

		}
		void release() override
		{
			
		}
		void render(QPainter* painter) override
		{

		}
	private:
	};
}
#endif