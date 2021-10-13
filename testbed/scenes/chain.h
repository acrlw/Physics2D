#ifndef PHYSICS2D_SCENES_CHAIN_H
#define PHYSICS2D_SCENES_CHAIN_H
#include "testbed/frame.h"
namespace Physics2D
{
	class ChainFrame : public Frame
	{
	public:
		ChainFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Chain", world, maintainer, tree, dbvh)
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