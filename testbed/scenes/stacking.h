#ifndef PHYSICS2D_SCENES_STACKING_H
#define PHYSICS2D_SCENES_STACKING_H
#include "testbed/frame.h"
namespace Physics2D
{
	class StackingFrame : public Frame
	{
	public:
		StackingFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Stacking", world, maintainer, tree, dbvh)
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