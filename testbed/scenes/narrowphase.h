#ifndef PHYSICS2D_SCENES_NARROWPHASE_H
#define PHYSICS2D_SCENES_NARROWPHASE_H
#include "testbed/frame.h"
namespace Physics2D
{
	class NarrowphaseFrame : public Frame
	{
	public:
		NarrowphaseFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Narrowphase", world, maintainer, tree, dbvh)
		{

		}
		void load() override
		{

		}
		void render(QPainter* painter) override
		{

		}
	private:

	};
}
#endif