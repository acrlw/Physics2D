#ifndef PHYSICS2D_TESTBED_SCENES_NEWTONCRADLE_H
#define PHYSICS2D_TESTBED_SCENES_NEWTONCRADLE_H

#include "testbed/frame.h"
namespace Physics2D
{
	class NewtonCradleFrame : public Frame
	{
	public:
		NewtonCradleFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Newton's Cradle", world, maintainer, tree, dbvh)
		{

		}
		void load() override
		{

		}

		void render(QPainter* painter) override
		{

		}
	private:
		std::unique_ptr<Circle> circle_ptr;
		std::unique_ptr<Edge> edge_ptr;

	};
}

#endif // !PHYSICS2D_TESTBED_SCENES_NEWTONCRADLE_H

