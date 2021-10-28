#ifndef PHYSICS2D_SCENES_FRICTION_H
#define PHYSICS2D_SCENES_FRICTION_H
#include "testbed/frame.h"
namespace Physics2D
{
	class FrictionFrame : public Frame
	{
	public:
		FrictionFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Friction", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			edge.set({ 0, 0 }, { 10, 0 });
			
			ramp.set({ -10, 4 }, { 0, 0 });

			rectangle.set(0.5f, 0.5f);
			
			for(real i = 0;i < 3.0f;i += 1.0f)
			{
				Body* ground = m_world->createBody();
				ground->setShape(&edge);
				ground->setFriction(0.1f);
				ground->setMass(Constant::Max);
				ground->position().set(0, i * 3.0f);
				ground->setRestitution(0);
				ground->setType(Body::BodyType::Static);

				Body* rampBody = m_world->createBody();
				rampBody->setShape(&ramp);
				rampBody->setFriction(0.1f);
				rampBody->setMass(Constant::Max);
				rampBody->position().set(0, i * 3.0f);
				rampBody->setRestitution(0);
				rampBody->setType(Body::BodyType::Static);

				m_tree->insert(ground);
				m_tree->insert(rampBody);
			}

			for (real i = 1; i < 4.0f; i += 1.0f)
			{
				Body* cube = m_world->createBody();
				cube->setShape(&rectangle);
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
		Rectangle rectangle;
		Edge edge;
		Edge ramp;
	};
}
#endif