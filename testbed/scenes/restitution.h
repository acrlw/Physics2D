#ifndef PHYSICS2D_SCENES_RESTITUTION_H
#define PHYSICS2D_SCENES_RESTITUTION_H
#include "testbed/frame.h"
namespace Physics2D
{
	class RestitutionFrame : public Frame
	{
	public:
		RestitutionFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Restitution", world, maintainer, tree, dbvh)
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
#endif