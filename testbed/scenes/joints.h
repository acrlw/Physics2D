#ifndef PHYSICS2D_SCENES_JOINTS_H
#define PHYSICS2D_SCENES_JOINTS_H
#include "testbed/frame.h"
namespace Physics2D
{
	class JointsFrame : public Frame
	{
	public:
		JointsFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Joints", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			capsule.set(1.5f, 0.5f);
			triangle.append({ {-1.0f, 1.0f},{0.0f, -2.0f},{1.0f, -1.0f},{-1.0f, 1.0f} });
			triangle.scale(0.5f);
			edge.set({ -100, 0 }, { 100, 0 });

			bodyA = m_world->createBody();
			bodyA->setShape(&triangle);
			bodyA->setMass(1.0f);
			bodyA->setType(Body::BodyType::Dynamic);
			bodyA->position().set(0, 2.0f);
			m_tree->insert(bodyA);

			bodyB = m_world->createBody();
			bodyB->setShape(&capsule);
			bodyB->setMass(1.0f);
			bodyB->setType(Body::BodyType::Dynamic);
			bodyB->position().set(0, 0.0f);
			m_tree->insert(bodyB);

			RevoluteJointPrimitive rjp;
			rjp.bodyA = bodyA;
			rjp.bodyB = bodyB;
			rjp.dampingRatio = 0.8f;
			rjp.frequency = 10;
			rjp.maxForce = 10000;
			
			joint = m_world->createJoint(rjp);

			updateJoint();


			Body* ground = m_world->createBody();
			ground->setShape(&edge);
			ground->position().set({ 0, -2.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_tree->insert(ground);
		}
		void render(QPainter* painter) override
		{
			QPen pen(Qt::darkCyan, 1, Qt::DashDotLine);
			Vector2 p = bodyA->toWorldPoint(joint->primitive().localPointA);
			RendererQtImpl::renderLine(painter, m_camera, p - 0.5f * distance * normal, p + 0.5f * distance * normal, pen);
		}
		void release() override
		{
		}
		void update(real dt) override
		{
			updateJoint();
		}
		
	private:
		void updateJoint()
		{
			auto pair = Detector::distance(bodyA, bodyB);
			normal = (pair.pointA - pair.pointB).normalize();
			Vector2 pb = pair.pointB + 0.5f * distance * normal;
			Vector2 pa = pair.pointA - 0.5f * distance * normal;
			joint->primitive().localPointA = bodyA->toLocalPoint(pa);
			joint->primitive().localPointB = bodyB->toLocalPoint(pb);
		}
		RevoluteJoint* joint;
		Body* bodyA;
		Body* bodyB;
		Capsule capsule;
		Polygon triangle;
		Edge edge;
		real distance = 2.0f;
		Vector2 normal;
	};
}
#endif