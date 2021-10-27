#ifndef PHYSICS2D_SCENES_RAYCAST_H
#define PHYSICS2D_SCENES_RAYCAST_H
#include "testbed/frame.h"
namespace Physics2D
{
	class RaycastFrame : public Frame
	{
	public:
		RaycastFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Raycast", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			rectangle.set(0.5f, 0.5f);
			circle.setRadius(0.5f);
			capsule.set(1.5f, 0.5f);
			triangle.append({ {-1.0f, 1.0f},{0.0f, -2.0f},{1.0f, -1.0f},{-1.0f, 1.0f} });
			polygon.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f },{0.0f, 4.0f } });
			triangle.scale(0.5f);
			polygon.scale(0.1f);


			Shape* shapeArray[5];
			shapeArray[0] = &rectangle;
			shapeArray[1] = &circle;
			shapeArray[2] = &triangle;
			shapeArray[3] = &polygon;
			shapeArray[4] = &capsule;

			for(int i = 0;i < 100; i++)
			{
				Body* body = m_world->createBody();
				body->position().set(-9.0f + QRandomGenerator::global()->bounded(18.0f),
					-9.0f + QRandomGenerator::global()->bounded(18.0f));
				body->setShape(shapeArray[QRandomGenerator::global()->bounded(0, 5)]);
				body->rotation() = -Constant::Pi + QRandomGenerator::global()->bounded(Constant::Pi);
				body->setMass(1);
				body->setType(Body::BodyType::Static);

				m_tree->insert(body);
			}

		}
		void render(QPainter* painter) override
		{
			Vector2 p;
			Vector2 d = m_mousePos.normal();
			QPen origin(Qt::gray, 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			QPen dir(Qt::green, 1, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin);
			QPen hit(Qt::cyan, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			RendererQtImpl::renderPoint(painter, m_camera, Vector2(0, 0), origin);
			RendererQtImpl::renderLine(painter, m_camera, p, d * 10.0f, dir);
			auto bodyList = m_tree->raycast(p, d);
			for(auto& elem: bodyList)
			{
				ShapePrimitive sp;
				sp.rotation = elem->rotation();
				sp.transform = elem->position();
				sp.shape = elem->shape();
				RendererQtImpl::renderShape(painter, m_camera, sp, hit);
			}

		}
		void onMouseMove(QMouseEvent* e) override
		{
			m_mousePos  = m_camera->screenToWorld(Vector2(e->pos().x(), e->pos().y()));
		}
	private:
		Vector2 m_mousePos = Vector2(1, 1);
		Rectangle rectangle;
		Circle circle;
		Polygon polygon;
		Capsule capsule;
		Polygon triangle;
	};
}
#endif