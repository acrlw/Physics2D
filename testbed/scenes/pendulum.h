#ifndef PHYSICS2D_TESTBED_SCENES_PENDULUM_H
#define PHYSICS2D_TESTBED_SCENES_PENDULUM_H

#include "testbed/frame.h"
#include <deque>
namespace Physics2D
{
	class PendulumFrame : public Frame
	{
	public:
		PendulumFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Pendulum", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			points.resize(400);

			uint32_t mask = 0x01;
			rectangle.set(4.0f, 0.25f);

			stick1 = m_world->createBody();
			stick1->setShape(&rectangle);
			stick1->setMass(2.0f);
			stick1->setBitmask(mask << 1);
			stick1->setType(Body::BodyType::Dynamic);
			stick1->position().set(0, 0);

			stick2 = m_world->createBody();
			stick2->setShape(&rectangle);
			stick2->setMass(2.0f);
			stick2->setBitmask(mask << 2);
			stick2->setType(Body::BodyType::Dynamic);
			real h = 1.5f * Math::fastInverseSqrt(2.0f);
			stick2->position().set(1.5f + h, h);
			stick2->rotation() = Math::degreeToRadian(45);

			RevoluteJointPrimitive rjp;
			rjp.bodyA = stick1;
			rjp.bodyB = stick2;
			rjp.localPointA.set(1.5f, 0);
			rjp.localPointB.set(-1.5f, 0);
			rjp.frequency = 10;
			rjp.dampingRatio = 0.8f;
			m_world->createJoint(rjp);

			PointJointPrimitive pjp;
			pjp.bodyA = stick1;
			pjp.frequency = 50;
			pjp.dampingRatio = 0.8f;
			pjp.maxForce = 10000;
			pjp.targetPoint.set(-1.5f, 0);
			pjp.localPointA.set(-1.5f, 0);
			m_world->createJoint(pjp);

			m_tree->insert(stick1);
			m_tree->insert(stick2);

			lvd = m_world->linearVelocityDamping();
			avd = m_world->angularVelocityDamping();
			m_world->setLinearVelocityDamping(0.0f);
			m_world->setAngularVelocityDamping(0.0f);
		}
		void render(QPainter* painter) override
		{
			if (points.size() > 400)
				points.pop_front();
			points.emplace_back(stick2->position());

			QPen p(Qt::cyan, 1);
			for(auto& elem: points)
				RendererQtImpl::renderPoint(painter, m_camera, elem, p);
		}
		void release() override
		{
			m_world->setLinearVelocityDamping(lvd);
			m_world->setAngularVelocityDamping(avd);
		}
	private:
		Body* stick1 = nullptr;
		Body* stick2 = nullptr;
		std::deque<Vector2> points;
		Rectangle rectangle;
		real lvd;
		real avd;

	};
}

#endif // !1
