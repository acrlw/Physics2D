#ifndef PHYSICS2D_SCENES_BRIDGE_H
#define PHYSICS2D_SCENES_BRIDGE_H
#include "testbed/frame.h"
namespace Physics2D
{
	class BridgeFrame : public Frame
	{
	public:
		BridgeFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Bridge", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			brick.set(1.5f, 0.5f);

			edge.set({ -100, 0 }, { 100, 0 });

			Body* rect;
			Body* rect2;
			Body* ground;

			real half = brick.width() / 2.0f;
			rect = m_world->createBody();
			rect->setShape(&brick);
			rect->position().set({ -15.0f, 0.0f });
			rect->rotation() = 0;
			rect->setMass(1.0f);
			rect->setRestitution(0.2f);
			rect->setFriction(0.01f);
			rect->setType(Body::BodyType::Dynamic);

			ground = m_world->createBody();
			ground->setShape(&edge);
			ground->position().set({ 0, -15.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_tree->insert(ground);

			PointJointPrimitive ppm;
			RevoluteJointPrimitive revolutePrim;

			ppm.bodyA = rect;
			ppm.localPointA.set(-half, 0);
			ppm.targetPoint.set(-15.0f - half, 0.0f);
			ppm.dampingRatio = 0.1f;
			ppm.frequency = 1000;
			ppm.maxForce = 10000;
			m_world->createJoint(ppm);
			real max = 20.0f;
			m_tree->insert(rect);
			for (real i = 1.0f; i < max; i += 1.0f)
			{
				rect2 = m_world->createBody();
				rect2->setShape(&brick);
				rect2->position().set({ -15.0f + i * brick.width() * 1.2f, 0.0f });
				rect2->rotation() = 0;
				rect2->setMass(1.0f);
				rect2->setFriction(0.01f);
				rect2->setType(Body::BodyType::Dynamic);

				this->m_tree->insert(rect2);
				revolutePrim.bodyA = rect;
				revolutePrim.bodyB = rect2;
				revolutePrim.localPointA.set(half + brick.width() * 0.1f, 0);
				revolutePrim.localPointB.set(-half - brick.width() * 0.1f, 0);
				revolutePrim.dampingRatio = 0.8f;
				revolutePrim.frequency = 10;
				revolutePrim.maxForce = 10000;
				m_world->createJoint(revolutePrim);
				rect = rect2;
			}

			ppm.bodyA = rect2;
			ppm.localPointA.set(0.75f, 0);
			ppm.targetPoint.set(rect2->toWorldPoint(Vector2(0.75f, 0.0f)));
			ppm.dampingRatio = 0.1f;
			ppm.frequency = 1000;
			ppm.maxForce = 10000;
			m_world->createJoint(ppm);

		}
		void render(QPainter* painter) override
		{

		}
	private:
		Rectangle brick;
		Edge edge;
	};
}
#endif