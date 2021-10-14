#ifndef PHYSICS2D_SCENES_STACKING_H
#define PHYSICS2D_SCENES_STACKING_H
#include "testbed/frame.h"
namespace Physics2D
{
	class StackingFrame : public Frame
	{
	public:
		StackingFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Stacking", world, maintainer, tree, dbvh)
		{

		}
		void load() override
		{
			Edge edge;
			edge.set({ -100, 0 }, { 100, 0 });
			edge_ptr = std::make_unique<Edge>(edge);

			Rectangle rectangle;
			rectangle.set(1.0f, 1.0f);
			rectangle_ptr = std::make_unique<Rectangle>(rectangle);

			Body* ground;

			ground = m_world->createBody();
			ground->setShape(edge_ptr.get());
			ground->position().set({ 0.0, 0.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_tree->insert(ground);

			real offset = 0.0f;
			real max = 20.0;
			for (real j = 0; j < max; j += 1.0f)
			{
				for (real i = 0.0; i < max - j; i += 1.0f)
				{
					Body* body = m_world->createBody();
					body->position().set({ -10.0f + i * 1.1f + offset, j * 1.8f + 2.0f });
					body->setShape(rectangle_ptr.get());
					body->rotation() = 0;
					body->setMass(1.0f);
					body->setType(Body::BodyType::Dynamic);
					body->setFriction(0.8f);
					body->setRestitution(0.0f);
					m_tree->insert(body);
				}
				offset += 0.5f;
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