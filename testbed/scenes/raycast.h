#ifndef PHYSICS2D_SCENES_RAYCAST_H
#define PHYSICS2D_SCENES_RAYCAST_H
#include "testbed/frame.h"
namespace Physics2D
{
	class RaycastFrame : public Frame
	{
	public:
		RaycastFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Raycast", world, maintainer, tree, dbvh)
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