#ifndef PHYSICS2D_SCENES_GEOMETRY_H
#define PHYSICS2D_SCENES_GEOMETRY_H
#include "testbed/frame.h"
namespace Physics2D
{
	class GeometryFrame : public Frame
	{
	public:
		GeometryFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Geometry", world, maintainer, tree, dbvh, camera)
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