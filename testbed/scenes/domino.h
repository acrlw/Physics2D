#ifndef PHYSICS2D_SCENES_DOMINO_H
#define PHYSICS2D_SCENES_DOMINO_H
#include "testbed/frame.h"
namespace Physics2D
{
	class DominoFrame : public Frame
	{
	public:
		DominoFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Domino", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			Rectangle floor(15.0f, 0.5f);
			Rectangle rect(0.5f, 0.5f);
			Rectangle brick(0.3f, 3.0f);
			Edge edge;
			edge.set(Vector2{ -100.0f, 0 }, Vector2{ 100.0f, 0 });
			brick_ptr = std::make_unique<Rectangle>(brick);
			rectangle_ptr = std::make_unique<Rectangle>(rect);
			floor_ptr = std::make_unique<Rectangle>(floor);
			edge_ptr = std::make_unique<Edge>(edge);

			Body* ground = m_world->createBody();
			ground->setShape(edge_ptr.get());
			ground->setType(Body::BodyType::Static);
			ground->setMass(Constant::Max);
			ground->position().set({ 0, 0.0f });
			ground->setFriction(0.1f);
			ground->setRestitution(0.0f);
			m_tree->insert(ground);

			Body* tile = m_world->createBody();
			tile->setShape(floor_ptr.get());
			tile->setType(Body::BodyType::Static);
			tile->setMass(Constant::Max);
			tile->setFriction(0.1f);
			tile->setRestitution(0.0f);
			tile->rotation() = Math::degreeToRadian(15);
			tile->position().set({ 4, 8 });
			m_tree->insert(tile);

			tile = m_world->createBody();
			tile->setShape(floor_ptr.get());
			tile->setType(Body::BodyType::Static);
			tile->setMass(Constant::Max);
			tile->setFriction(0.1f);
			tile->setRestitution(0.0f);
			tile->rotation() = Math::degreeToRadian(-15);
			tile->position().set({ -4, 4 });
			m_tree->insert(tile);


			tile = m_world->createBody();
			tile->setShape(floor_ptr.get());
			tile->setType(Body::BodyType::Static);
			tile->setMass(Constant::Max);
			tile->setFriction(0.1f);
			tile->setRestitution(0.0f);
			tile->rotation() = 0;
			tile->position().set({ -5, 10 });
			m_tree->insert(tile);

			for(real i = 0;i < 9.0; i += 1.0f)
			{
				Body* card = m_world->createBody();
				card->setShape(brick_ptr.get());
				card->setMass(1.0f);
				card->setFriction(0.1f);
				card->setRestitution(0);
				card->setType(Body::BodyType::Dynamic);
				card->position().set({ -10.0f + i * 1.2f, 12.0f });
				m_tree->insert(card);
			}

			Body* stammer = m_world->createBody();
			stammer->setShape(rectangle_ptr.get());
			stammer->setMass(10.0f);
			stammer->setFriction(0.1f);
			stammer->setType(Body::BodyType::Dynamic);
			stammer->position().set(-16.0f, 16.0f);
			m_tree->insert(stammer);

			DistanceJointPrimitive djp;
			djp.bodyA = stammer;
			djp.localPointA.set(0, 0);
			djp.minDistance = 1.0f;
			djp.maxDistance = 4.0f;
			djp.targetPoint.set(-12.0f, 16.0f);
			m_world->createJoint(djp);

			OrientationJointPrimitive ojp;
			ojp.targetPoint.set(-12.0f, 16.0f);
			ojp.bodyA = stammer;
			ojp.referenceRotation = 0;
			m_world->createJoint(ojp);



		}
		void render(QPainter* painter) override
		{

		}
	private:
		std::unique_ptr<Rectangle> brick_ptr;
		std::unique_ptr<Rectangle> floor_ptr;
		std::unique_ptr<Edge> edge_ptr;
		std::unique_ptr<Rectangle> rectangle_ptr;
	};
}
#endif