#ifndef PHYSICS2D_SCENES_JOINTS_H
#define PHYSICS2D_SCENES_JOINTS_H
#include "testbed/frame.h"
namespace Physics2D
{
	class JointsFrame : public Frame
	{
	public:
		JointsFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Joints", world, maintainer, tree, dbvh)
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