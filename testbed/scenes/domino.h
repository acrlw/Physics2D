#ifndef PHYSICS2D_SCENES_DOMINO_H
#define PHYSICS2D_SCENES_DOMINO_H
#include "testbed/frame.h"
namespace Physics2D
{
	class DominoFrame : public Frame
	{
	public:
		DominoFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Domino", world, maintainer, tree, dbvh)
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