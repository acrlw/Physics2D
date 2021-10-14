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
			Edge edge;
			edge.set({ 0, 0 }, { 10, 0 });
			edge_ptr = std::make_unique<Edge>(edge);

			Edge ramp;
			ramp.set({ -10, 4 }, { 0, 0 });
			ramp_ptr = std::make_unique<Edge>(ramp);


			Rectangle rect(0.5f, 0.5f);
			rectangle_ptr = std::make_unique<Rectangle>(rect);

			for(real i = 0;i < 3.0f;i += 1.0f)
			{
				Body* ground = m_world->createBody();
				ground->setShape(edge_ptr.get());
				ground->setFriction(0.1f);
				ground->setMass(Constant::Max);
				ground->position().set(0, i * 3.0f);
				ground->setRestitution(0);
				ground->setType(Body::BodyType::Static);

				Body* ramp = m_world->createBody();
				ramp->setShape(ramp_ptr.get());
				ramp->setFriction(0.1f);
				ramp->setMass(Constant::Max);
				ramp->position().set(0, i * 3.0f);
				ramp->setRestitution(0);
				ramp->setType(Body::BodyType::Static);

				m_tree->insert(ground);
				m_tree->insert(ramp);
			}

			for (real i = 1; i < 4.0f; i += 1.0f)
			{
				Body* cube = m_world->createBody();
				cube->setShape(rectangle_ptr.get());
				cube->setFriction(i * 0.3f);
				cube->setMass(1);
				cube->position().set(-5.0f, i * 3.5f);
				cube->setRestitution(0);
				cube->setType(Body::BodyType::Dynamic);
				m_tree->insert(cube);
			}


		}
		void render(QPainter* painter) override
		{

		}
	private:
		std::unique_ptr<Rectangle> rectangle_ptr;
		std::unique_ptr<Edge> edge_ptr;
		std::unique_ptr<Edge> ramp_ptr;
	};
}
#endif