#ifndef PHYSICS2D_SCENES_GEOMETRY_H
#define PHYSICS2D_SCENES_GEOMETRY_H
#include "testbed/frame.h"
namespace Physics2D
{
	class GeometryFrame : public Frame
	{
	public:
		GeometryFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Geometry", world, maintainer, tree, dbvh)
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