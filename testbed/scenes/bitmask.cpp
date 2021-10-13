#ifndef PHYSICS2D_SCENES_BITMASK_H
#define PHYSICS2D_SCENES_BITMASK_H
#include "testbed/frame.h"
namespace Physics2D
{
	class BitmaskFrame : public Frame
	{
	public:
		BitmaskFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Bitmask", world, maintainer, tree, dbvh)
		{
			
		}
		void load() override
		{
			
		}
	private:
	};
}
#endif