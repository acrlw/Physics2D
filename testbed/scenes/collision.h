#ifndef PHYSICS2D_SCENES_COLLISION_H
#define PHYSICS2D_SCENES_COLLISION_H
#include "testbed/frame.h"
namespace Physics2D
{
	class CollisionFrame : public Frame
	{
	public:
		CollisionFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Collision", world, maintainer, tree, dbvh)
		{

		}
		void load() override
		{
			Edge edge;
			edge.set({ -100, 5 }, { 100, 0 });
			edge_ptr = std::make_unique<Edge>(edge);

			Rectangle rectangle;
			rectangle.set(1.0f, 1.0f);
			rectangle_ptr = std::make_unique<Rectangle>(rectangle);

			Body* ground;
			Body* rect;

			ground = m_world->createBody();
			ground->setShape(edge_ptr.get());
			ground->position().set({ 0, 0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			ground->setFriction(0.7f);
			ground->setRestitution(1.0);
			m_tree->insert(ground);

			rect = m_world->createBody();
			rect->setShape(rectangle_ptr.get());
			rect->position().set({ -5, 6 });
			rect->rotation() = 2.21805891827f;
			rect->setMass(1);
			rect->setType(Body::BodyType::Dynamic);
			rect->setFriction(0.4f);
			rect->setRestitution(0.0f);
			m_tree->insert(rect);
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