#ifndef PHYSICS2D_SCENES_BRIDGE_H
#define PHYSICS2D_SCENES_BRIDGE_H
#include "testbed/frame.h"
namespace Physics2D
{
	class BridgeFrame : public Frame
	{
	public:
		BridgeFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Bridge", world, maintainer, tree, dbvh)
		{

		}
		void release() override
		{
			brick_ptr.release();
		}
		void load() override
		{
			Rectangle brick(1.5f, 0.5f);
			brick_ptr = std::make_unique<Rectangle>(brick);

		}
		void render(QPainter* painter) override
		{

		}
	private:
		std::unique_ptr<Rectangle> brick_ptr;
	};
}
#endif