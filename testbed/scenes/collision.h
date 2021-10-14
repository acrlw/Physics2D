#ifndef PHYSICS2D_SCENES_COLLISION_H
#define PHYSICS2D_SCENES_COLLISION_H
#include "testbed/frame.h"
namespace Physics2D
{
	class CollisionFrame : public Frame
	{
	public:
		CollisionFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Collision", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			Edge edge;
			edge.set({ -10, 0 }, { 10, 0 });
			edge_ptr = std::make_unique<Edge>(edge);

			Capsule capsule;
			capsule.set(2.0f, 1.0f);
			capsule_ptr = std::make_unique<Capsule>(capsule);

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
			rect->setShape(capsule_ptr.get());
			rect->position().set({ 0, 6 });
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
		std::unique_ptr<Capsule> capsule_ptr;
		std::unique_ptr<Edge> edge_ptr;

	};
}
#endif