#ifndef PHYSICS2D_SCENES_WRECKINGBALL_H
#define PHYSICS2D_SCENES_WRECKINGBALL_H
#include "testbed/frame.h"
namespace Physics2D
{
	class WreckingBallFrame : public Frame
	{
	public:
		WreckingBallFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Wrecking Ball", world, maintainer, tree, dbvh, camera)
		{

		}

		void load() override
		{
			Body* rect;
			Body* rect2;
			Body* ground;

			rectangle.set(1.0f, 1.0f);
			circle.setRadius(1.5f);
			brick.set(1.5f, 0.5f);
			edge.set({ -100, 0 }, { 100, 0 });


			DistanceJointPrimitive distancePrim;


			ground = m_world->createBody();
			ground->setShape(&edge);
			ground->position().set({ 0, 0.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_tree->insert(ground);

			for (real j = 0; j < 15.0f; j += 1.0f)
			{
				for (real i = 0; i < 6.0; i += 1.0f)
				{
					Body* body = m_world->createBody();
					body->position().set({ i * 1.05f - 32.0f, j * 1.05f - ground->position().y + 0.55f });
					body->setShape(&rectangle);
					body->rotation() = 0.0f;
					body->setMass(0.1f);
					body->setType(Body::BodyType::Dynamic);
					body->setFriction(0.8f);
					body->setRestitution(0.0f);
					m_tree->insert(body);
				}
			}
			for (real j = 0; j < 15.0f; j += 1.0f)
			{
				for (real i = 0; i < 6.0; i += 1.0f)
				{
					Body* body = m_world->createBody();
					body->position().set({ i * 1.05f - 2.0f, j * 1.05f - ground->position().y + 0.55f });
					body->setShape(&rectangle);
					body->rotation() = 0.0f;
					body->setMass(0.1f);
					body->setType(Body::BodyType::Dynamic);
					body->setFriction(0.8f);
					body->setRestitution(0.0f);
					m_tree->insert(body);
				}
			}

			real half = brick.width() / 2.0f;
			rect = m_world->createBody();
			rect->setShape(&brick);
			rect->position().set({ -20.0f, 20.0f });
			rect->rotation() = 0;
			rect->setMass(1.0f);
			rect->setRestitution(0.2f);
			rect->setFriction(0.8f);
			rect->setType(Body::BodyType::Dynamic);


			PointJointPrimitive ppm;
			ppm.bodyA = rect;
			ppm.localPointA.set(-half, 0);
			ppm.targetPoint.set(-20.0f - half, 20.0f);
			ppm.dampingRatio = 0.8f;
			ppm.frequency = 1000;
			ppm.maxForce = 10000;
			m_world->createJoint(ppm);
			real max = 7.0f;
			m_tree->insert(rect);
			for (real i = 1.0f; i < max; i += 1.0f)
			{
				rect2 = m_world->createBody();
				rect2->setShape(&brick);
				rect2->position().set({ -20.0f + i * brick.width(), 20.0f });
				rect2->rotation() = 0;
				rect2->setMass(1.0f);
				rect2->setFriction(0.1f);
				rect2->setType(Body::BodyType::Dynamic);

				m_tree->insert(rect2);
				RevoluteJointPrimitive revolutePrim;
				revolutePrim.bodyA = rect;
				revolutePrim.bodyB = rect2;
				revolutePrim.localPointA.set(half, 0);
				revolutePrim.localPointB.set(-half, 0);
				revolutePrim.dampingRatio = 0.8f;
				revolutePrim.frequency = 20;
				revolutePrim.maxForce = Constant::Max;
				m_world->createJoint(revolutePrim);
				rect = rect2;
			}
			rect2 = m_world->createBody();
			rect2->setShape(&circle);
			rect2->position().set({ -20.0f + max * brick.width() + half, 20.0f });
			rect2->rotation() = 0;
			rect2->setMass(25.0f);
			rect2->setFriction(0.1f);
			rect2->setType(Body::BodyType::Dynamic);

			m_tree->insert(rect2);
			RevoluteJointPrimitive revolutePrim;
			revolutePrim.bodyA = rect;
			revolutePrim.bodyB = rect2;
			revolutePrim.localPointA.set(half, 0);
			revolutePrim.localPointB.set(-half * 2.0f, 0);
			revolutePrim.dampingRatio = 0.8f;
			revolutePrim.frequency = 20;
			revolutePrim.maxForce = Constant::Max;
			m_world->createJoint(revolutePrim);


			rect2 = m_world->createBody();
			rect2->setShape(&circle);
			rect2->position().set({ 21.5f + half, 20.0f });
			rect2->rotation() = 0;
			rect2->setMass(25.0f);
			rect2->setFriction(0.1f);
			rect2->setType(Body::BodyType::Dynamic);
			m_tree->insert(rect2);

			distancePrim.bodyA = rect2;
			distancePrim.localPointA.set({ 0, 0 });
			distancePrim.minDistance = 11.5f + half;
			distancePrim.maxDistance = 11.5f + half;
			distancePrim.targetPoint.set({ 10, 20 });

			m_world->createJoint(distancePrim);

		}
		void render(QPainter* painter) override
		{

		}
	private:
		Rectangle rectangle;
		Rectangle brick;
		Edge edge;
		Circle circle;

	};
}
#endif