#ifndef PHYSICS2D_SCENES_STACKING_H
#define PHYSICS2D_SCENES_STACKING_H
#include "testbed/frame.h"
namespace Physics2D
{
	class StackingFrame : public Frame
	{
	public:
		StackingFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Stacking", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			edge.set({ -100, 0 }, { 100, 0 });
			rectangle.set(1.0f, 1.0f);

			Body* ground;

			ground = m_world->createBody();
			ground->setShape(&edge);
			ground->position().set({ 0.0, 0.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_tree->insert(ground);

			real offset = 0.0f;
			real max = 25.0;
			for (real j = 0; j < max; j += 1.0f)
			{
				for (real i = 0.0; i < max - j; i += 1.0f)
				{
					Body* body = m_world->createBody();
					body->position().set({ -10.0f + i * 1.1f + offset, j * 1.8f + 2.0f });
					body->setShape(&rectangle);
					body->rotation() = 0;
					body->setMass(0.2f);
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
		Rectangle rectangle;
		Edge edge;

	};
}
#endif