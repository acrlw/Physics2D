#ifndef PHYSICS2D_SCENES_COLLISION_H
#define PHYSICS2D_SCENES_COLLISION_H
#include "testbed/frame.h"
namespace Physics2D
{
	class CollisionFrame : public Frame
	{
	public:
		CollisionFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Collision", world, maintainer, tree, dbvh)
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