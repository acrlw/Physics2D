#ifndef PHYSICS2D_TESTBED_SCENES_NEWTONCRADLE_H
#define PHYSICS2D_TESTBED_SCENES_NEWTONCRADLE_H

#include "testbed/frame.h"
namespace Physics2D
{
	class NewtonCradleFrame : public Frame
	{
	public:
		NewtonCradleFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Newton's Cradle", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			real startX = -5.0f;
			Circle circle(1.0f);
			circle_ptr = std::make_unique<Circle>(circle);
			DistanceJointPrimitive djp;
			djp.minDistance = 4.9f;
			djp.maxDistance = 5.0f;
			djp.localPointA.set(0, 0);

			Body* ball = m_world->createBody();
			ball->setShape(circle_ptr.get());
			ball->setMass(1.0f);
			ball->setType(Body::BodyType::Dynamic);
			ball->position().set(startX, 5.0);
			ball->setRestitution(1.0f);

			djp.targetPoint.set(startX, 10.0f);
			djp.bodyA = ball;
			m_world->createJoint(djp);

			m_tree->insert(ball);


			for(real i = 0;i < 5.0f; i++)
			{
				startX += 2.01f;
				ball = m_world->createBody();
				ball->setShape(circle_ptr.get());
				ball->setMass(1.0f);
				ball->setType(Body::BodyType::Dynamic);
				ball->setFriction(0.1f);
				ball->setRestitution(1.0f);
				ball->position().set(startX, 5.0);
				djp.targetPoint.set(startX, 10.0f);
				djp.bodyA = ball;
				m_world->createJoint(djp);
				m_tree->insert(ball);
			}

			startX += 2.01f;
			ball = m_world->createBody();
			ball->setShape(circle_ptr.get());
			ball->setMass(1.0f);
			ball->setType(Body::BodyType::Dynamic);
			ball->position().set(startX + 5.0f, 10.0f);
			ball->setRestitution(1.0f);

			djp.targetPoint.set(startX, 10.0f);
			djp.bodyA = ball;
			m_world->createJoint(djp);

			m_tree->insert(ball);
		}

		void render(QPainter* painter) override
		{

		}
	private:
		std::unique_ptr<Circle> circle_ptr;

	};
}

#endif // !PHYSICS2D_TESTBED_SCENES_NEWTONCRADLE_H

