#ifndef PHYSICS2D_SCENES_RESTITUTION_H
#define PHYSICS2D_SCENES_RESTITUTION_H
#include "testbed/frame.h"
namespace Physics2D
{
	class RestitutionFrame : public Frame
	{
	public:
		RestitutionFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Restitution", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			circle.setRadius(1.0f);
			
			edge.set(Vector2{ -100.0, 0 }, Vector2{ 100.0, 0 });

			Body* ground = m_world->createBody();
			ground->setShape(&edge);
			ground->setMass(Constant::Max);
			ground->setRestitution(1.0f);
			ground->setFriction(0.9f);
			ground->position().set({ 0, 0 });
			ground->setType(Body::BodyType::Static);
			m_tree->insert(ground);

			for(real i = 0;i < 10.0f;i += 1.0f)
			{
				Body* body = m_world->createBody();
				body->setShape(&circle);
				body->setMass(10.0f);
				body->setFriction(0.1f);
				body->setRestitution(i / 10.0f);
				body->position().set(i * 2.5f - 10, 10.0f);
				body->setType(Body::BodyType::Dynamic);
				m_tree->insert(body);
			}
		}
		void render(QPainter* painter) override
		{

		}
	private:
		Circle circle;
		Edge edge;
	};
}
#endif