#ifndef PHYSICS2D_SCENES_WRECKINGBALL_H
#define PHYSICS2D_SCENES_WRECKINGBALL_H
#include "testbed/frame.h"
namespace Physics2D
{
	class WreckingBallFrame : public Frame
	{
	public:
		WreckingBallFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : Frame("WreckingBall", world, maintainer, tree, dbvh)
		{

		}

		void load() override
		{
			Body* rect;
			Body* rect2;
			Body* ground;

			Rectangle rectangle(1.0f, 1.0f);
			Circle circle(1.5f);
			Rectangle brick(1.5f, 0.5f);
			Edge edge;
			edge.set({ -100, 0 }, { 100, 0 });
			brick_ptr = std::make_unique<Rectangle>(brick);
			edge_ptr = std::make_unique<Edge>(edge);
			circle_ptr = std::make_unique<Circle>(circle);
			rectangle_ptr = std::make_unique<Rectangle>(rectangle);

			DistanceJointPrimitive distancePrim;


			ground = m_world->createBody();
			ground->setShape(edge_ptr.get());
			ground->position().set({ 0, 0.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_tree->insert(ground);

			for (real j = 0; j < 10.0f; j += 1.0f)
			{
				for (real i = 0; i < 5.0; i += 1.0f)
				{
					Body* body = m_world->createBody();
					body->position().set({ i - 30.0f, j * 1.0f - ground->position().y + 0.55f });
					body->setShape(rectangle_ptr.get());
					body->rotation() = 0.0f;
					body->setMass(1.0f);
					body->setType(Body::BodyType::Dynamic);
					body->setFriction(0.8f);
					body->setRestitution(0.0f);
					m_tree->insert(body);
				}
			}
			for (real j = 0; j < 10.0f; j += 1.0f)
			{
				for (real i = 0; i < 5.0; i += 1.0f)
				{
					Body* body = m_world->createBody();
					body->position().set({ i, j * 1.0f - ground->position().y + 0.55f });
					body->setShape(rectangle_ptr.get());
					body->rotation() = 0.0f;
					body->setMass(1.0f);
					body->setType(Body::BodyType::Dynamic);
					body->setFriction(0.8f);
					body->setRestitution(0.0f);
					m_tree->insert(body);
				}
			}

			real half = brick_ptr->width() / 2.0f;
			rect = m_world->createBody();
			rect->setShape(brick_ptr.get());
			rect->position().set({ -20.0f, 15.0f });
			rect->rotation() = 0;
			rect->setMass(1.0f);
			rect->setRestitution(0.2f);
			rect->setFriction(0.8f);
			rect->setType(Body::BodyType::Dynamic);


			PointJointPrimitive ppm;
			ppm.bodyA = rect;
			ppm.localPointA.set(-half, 0);
			ppm.targetPoint.set(-20.0f - half, 15.0f);
			ppm.dampingRatio = 0.8f;
			ppm.frequency = 1000;
			ppm.maxForce = 10000;
			m_world->createJoint(ppm);
			real max = 7.0f;
			m_tree->insert(rect);
			for (real i = 1.0f; i < max; i += 1.0f)
			{
				rect2 = m_world->createBody();
				rect2->setShape(brick_ptr.get());
				rect2->position().set({ -20.0f + i * brick_ptr->width(), 15.0f });
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
				revolutePrim.frequency = 10;
				revolutePrim.maxForce = 10000;
				m_world->createJoint(revolutePrim);
				rect = rect2;
			}
			rect2 = m_world->createBody();
			rect2->setShape(circle_ptr.get());
			rect2->position().set({ -20.0f + max * brick_ptr->width() + half, 15.0f });
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
			revolutePrim.dampingRatio = 0.1f;
			revolutePrim.frequency = 10;
			m_world->createJoint(revolutePrim);


			rect2 = m_world->createBody();
			rect2->setShape(circle_ptr.get());
			rect2->position().set({ 21.5f + half, 15.0f });
			rect2->rotation() = 0;
			rect2->setMass(25.0f);
			rect2->setFriction(0.1f);
			rect2->setType(Body::BodyType::Dynamic);
			m_tree->insert(rect2);

			distancePrim.bodyA = rect2;
			distancePrim.localPointA.set({ 0, 0 });
			distancePrim.minDistance = 3;
			distancePrim.maxDistance = 11.5f + half;
			distancePrim.targetPoint.set({ 10, 15 });

			m_world->createJoint(distancePrim);

		}
		void render(QPainter* painter) override
		{

		}
	private:
		std::unique_ptr<Rectangle> brick_ptr;
		std::unique_ptr<Rectangle> rectangle_ptr;
		std::unique_ptr<Edge> edge_ptr;
		std::unique_ptr<Circle> circle_ptr;

	};
}
#endif