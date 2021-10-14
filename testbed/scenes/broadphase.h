#ifndef PHYSICS2D_SCENES_BROADPHASE_H
#define PHYSICS2D_SCENES_BROADPHASE_H
#include <QRandomGenerator>

#include "testbed/frame.h"
namespace Physics2D
{
	class BroadPhaseFrame : public Frame
	{
	public:
		BroadPhaseFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Broad Phase", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			Rectangle rect(0.5f, 0.5f);
			Circle circle(0.5f);
			Capsule capsule(1.5f, 0.5f);
			Polygon triangle, polygon;
			triangle.append({ {-1.0f, 1.0f},{0.0f, -2.0f},{1.0f, -1.0f},{-1.0f, 1.0f} });
			polygon.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f },{0.0f, 4.0f } });
			triangle.scale(0.5f);
			polygon.scale(0.1f);

			rectangle_ptr = std::make_unique<Rectangle>(rect);
			circle_ptr = std::make_unique<Circle>(circle);
			triangle_ptr = std::make_unique<Polygon>(triangle);
			polygon_ptr = std::make_unique<Polygon>(polygon);
			capsule_ptr = std::make_unique<Capsule>(capsule);

			Shape* shapeArray[5];
			shapeArray[0] = rectangle_ptr.get();
			shapeArray[1] = circle_ptr.get();
			shapeArray[2] = triangle_ptr.get();
			shapeArray[3] = polygon_ptr.get();
			shapeArray[4] = capsule_ptr.get();

			for (int i = 0; i < 100; i++)
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
			AABB queryRegion;
			queryRegion.width = 8;
			queryRegion.height = 8;
			auto bodyList = m_tree->query(queryRegion);
			QPen region(QColor("#B2EBF2"), 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			QPen hit(QColor("#FF9800"), 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

			Rectangle rect(8, 8);
			ShapePrimitive sp;
			sp.shape = &rect;
			RendererQtImpl::renderShape(painter, m_camera, sp, region);

			for(auto& elem: bodyList)
				RendererQtImpl::renderAABB(painter, m_camera, elem->aabb(), hit);
		}
	private:
		std::unique_ptr<Rectangle> rectangle_ptr;
		std::unique_ptr<Circle> circle_ptr;
		std::unique_ptr<Polygon> polygon_ptr;
		std::unique_ptr<Capsule> capsule_ptr;
		std::unique_ptr<Polygon> triangle_ptr;
	};
}
#endif