#ifndef PHYSICS2D_SCENES_BRIDGE_H
#define PHYSICS2D_SCENES_BRIDGE_H
#include "testbed/frame.h"
namespace Physics2D
{
	class BridgeFrame : public Frame
	{
	public:
		BridgeFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("Bridge", world, maintainer, tree, dbvh)
		{

		}
		void load() override
		{
			Rectangle brick(1.5f, 0.5f);
			brick_ptr = std::make_unique<Rectangle>(brick);

			Edge edge;
			edge.set({ -100, 0 }, { 100, 0 });
			edge_ptr = std::make_unique<Edge>(edge);

			Body* rect;
			Body* rect2;
			Body* ground;

			real half = brick_ptr->width() / 2.0f;
			rect = m_world->createBody();
			rect->setShape(brick_ptr.get());
			rect->position().set({ -5.0f, 0.0f });
			rect->rotation() = 0;
			rect->setMass(1.0f);
			rect->setRestitution(0.2f);
			rect->setFriction(0.01f);
			rect->setType(Body::BodyType::Dynamic);

			ground = m_world->createBody();
			ground->setShape(edge_ptr.get());
			ground->position().set({ 0, -15.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_tree->insert(ground);

			PointJointPrimitive ppm;
			RevoluteJointPrimitive revolutePrim;

			ppm.bodyA = rect;
			ppm.localPointA.set(-half, 0);
			ppm.targetPoint.set(-5.0f - half, 0.0f);
			ppm.dampingRatio = 0.1f;
			ppm.frequency = 100;
			m_world->createJoint(ppm);
			real max = 12.0f;
			m_tree->insert(rect);
			for (real i = 1.0f; i < max; i += 1.0f)
			{
				rect2 = m_world->createBody();
				rect2->setShape(brick_ptr.get());
				rect2->position().set({ -5.0f + i * brick_ptr->width(), 0.0f });
				rect2->rotation() = 0;
				rect2->setMass(1.0f);
				rect2->setFriction(0.01f);
				rect2->setType(Body::BodyType::Dynamic);

				this->m_tree->insert(rect2);
				revolutePrim.bodyA = rect;
				revolutePrim.bodyB = rect2;
				revolutePrim.localPointA.set(half, 0);
				revolutePrim.localPointB.set(-half, 0);
				revolutePrim.dampingRatio = 0.1f;
				revolutePrim.frequency = 5;
				m_world->createJoint(revolutePrim);
				rect = rect2;
			}

			ppm.bodyA = rect2;
			ppm.localPointA.set(0.75f, 0);
			ppm.targetPoint.set(-5.0f + max * brick_ptr->width() - half, 0.0f);
			ppm.dampingRatio = 0.1f;
			ppm.frequency = 100;
			m_world->createJoint(ppm);

		}
		void render(QPainter* painter) override
		{

		}
	private:
		std::unique_ptr<Rectangle> brick_ptr;
		std::unique_ptr<Edge> edge_ptr;
	};
}
#endif