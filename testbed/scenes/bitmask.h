#ifndef PHYSICS2D_SCENES_BITMASK_H
#define PHYSICS2D_SCENES_BITMASK_H
#include "testbed/frame.h"
namespace Physics2D
{
	class BitmaskFrame : public Frame
	{
	public:
		BitmaskFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Bitmask", world, maintainer, tree, dbvh)
		{
			
		}
		void load() override
		{
			Rectangle rect(1.0f, 1.0f);
			Edge edge;
			edge.set(Vector2{ -10.0f, 0.0f }, Vector2{ 10.0f, 0.0f });

			edge_ptr = std::make_unique<Edge>(edge);
			rectangle_ptr = std::make_unique<Rectangle>(rect);
			uint32_t mask = 0x01;
			for(real i = 0;i < 3.0;i += 1.0f)
			{
				Body* ground = m_world->createBody();
				ground->setShape(edge_ptr.get());
				ground->position().set({ 0, -6 + i * 3.0f });
				ground->setFriction(0.9f);
				ground->setBitmask(mask);
				ground->setRestitution(0);
				ground->setMass(Constant::Max);
				ground->setType(Body::BodyType::Static);
				mask = mask << 1;
				m_tree->insert(ground);
			}
			mask = 0x01;
			for (real i = 0; i < 3.0; i += 1.0f)
			{
				Body* body = m_world->createBody();
				body->setShape(rectangle_ptr.get());
				body->position().set({ i * 3.0f, 6.0f });
				body->setFriction(0.9f);
				body->setBitmask(mask);
				body->setRestitution(0);
				body->setMass(1);
				body->setType(Body::BodyType::Dynamic);
				mask = mask << 1;
				m_tree->insert(body);
			}
		}
		void render(QPainter* painter) override
		{
			
		}
	private:
		std::unique_ptr<Rectangle> rectangle_ptr;
		std::unique_ptr<Edge> edge_ptr;
	};
}
#endif