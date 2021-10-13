#ifndef PHYSICS2D_SCENES_FRICTION_H
#define PHYSICS2D_SCENES_FRICTION_H
#include "testbed/frame.h"
namespace Physics2D
{
	class FrictionFrame : public Frame
	{
	public:
		FrictionFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Friction", world, maintainer, tree, dbvh)
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