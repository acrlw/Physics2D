#ifndef PHYSICS2D_SCENES_GEOMETRY_H
#define PHYSICS2D_SCENES_GEOMETRY_H
#include "testbed/frame.h"
#include <deque>
#include <iostream>
namespace Physics2D
{
	class GeometryFrame : public Frame
	{
	public:
		GeometryFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Utils::Camera* camera) : Frame("Geometry", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
		}
		void render(QPainter* painter) override
		{
			QPen accurate(Qt::red, 1);
			QPen enhancedEuler(Qt::cyan, 1);
			QPen rk4(Qt::green, 1);
			QPen adams4(Qt::magenta, 1);
			QPen implicitAdams4(Qt::blue, 1);
			real x = 0;
			real y = 1;
			real z = 2;
			real h = 0.1f;
			real k1, k2, k3, k4;
			real l1, l2, l3, l4;
			real yc, yp;
			std::vector<Vector2> points;
			std::vector<Vector2> pointsAccurate;
			//auto dz = [](real x, real y, real z)
			//{
			//	return -3 * z - 2 * y - x;
			//};
			//auto dy = [](real x, real y, real z)
			//{
			//	return z;
			//};
			//points.emplace_back(Vector2{ x, y });
			//for (int i = 0; i < 50; i++)
			//{
			//	k1 = dy(x, y, z);
			//	l1 = dz(x, y, z);
			//	k2 = dy(x + h / 2.0f, y + h / 2.0f * k1, z + h / 2.0f * l1);
			//	l2 = dz(x + h / 2.0f, y + h / 2.0f * k1, z + h / 2.0f * l1);
			//	k3 = dy(x + h / 2.0f, y + h / 2.0f * k2, z + h / 2.0f * l2);
			//	l3 = dz(x + h / 2.0f, y + h / 2.0f * k2, z + h / 2.0f * l2);
			//	k4 = dy(x + h, y + h * k3, z + h * l3);
			//	l4 = dz(x + h, y + h * k3, z + h * l3);
			//	y = y + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0f;
			//	z = z + h * (l1 + 2 * l2 + 2 * l3 + l4) / 6.0f;
			//	x += h;
			//	points.emplace_back(Vector2{ x, y });
			//}
			//RendererQtImpl::renderPoints(painter, m_camera, points, enhancedEuler);

			std::deque<real> arrayY;
			auto dy = [](real x, real y)
			{
				return -y - x * y * y;
			};
			arrayY.emplace_back(y);
			points.reserve(50);
			pointsAccurate.reserve(50);
			points.emplace_back(Vector2{ x, y });

			for (int i = 0; i < 3; i++)
			{
				pointsAccurate.emplace_back(Vector2{ x, 1.0f / (2.0f * std::exp(x) - x - 1) });
				//euler:
				//y = y + h * dy(x, y);
				
				//
				//midpoint euler:
				//yp = y + h * dy(x, y);
				//yc = y + h * dy(x + h, yp);
				//y = (yp + yc) / 2.0f;

				//rk4:
				k1 = dy(x, y);
				k2 = dy(x + k1 * h / 2.0f, y);
				k3 = dy(x + k2 * h / 2.0f, y);
				k4 = dy(x + k3 * h, y);
				y = y + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0f;

				x += h;
				points.emplace_back(Vector2{ x, y });
				arrayY.emplace_back(y);
			}
			for (int i = 0; i < 50; i++)
			{
				pointsAccurate.emplace_back(Vector2{ x, 1.0f / (2.0f * std::exp(x) - x - 1) });
				//y = y + h * dy(x, y);
				//yp = y + h * dy(x, y);
				//yc = y + h * dy(x + h, yp);
				//y = (yp + yc) / 2.0f;
				
				//explicit adams4:
				y = y + (h / 24.0f) * (55 * dy(x, arrayY[3]) - 59 * dy(x - h * 1, arrayY[2]) + 37 * dy(x - h * 2, arrayY[1]) - 9 * dy(x - h * 3, arrayY[0]));
				//implicit adams4:
				//y = y + (h / 24.0f) * (9 * dy(x, arrayY[3]) + 19 * dy(x - h * 1, arrayY[2]) - 5 * dy(x - h * 2, arrayY[1]) + dy(x - h * 3, arrayY[0]));
				x += h;
				arrayY.pop_front();
				arrayY.emplace_back(y);
				points.emplace_back(Vector2{ x, y });
			}
			RendererQtImpl::renderPoints(painter, m_camera, points, enhancedEuler);
			RendererQtImpl::renderPoints(painter, m_camera, pointsAccurate, accurate);
			//pointsAccurate.clear();
			//points.clear();
			//
			
			//real t = 0;
			//real y1 = 0;
			//real y2 = 1;
			//auto dy1 = [](real y1, real y2)
			//{
			//	return 3 * y1 + 2 * y2;
			//};
			//auto dy2 = [](real y1, real y2)
			//{
			//	return 4 * y1 + y2;
			//};
			//for (int i = 0; i < 4; i++)
			//{
			//	std::cout << "t=" << t << " y1=" << y1 << " y2=" << y2 << std::endl;
			//	points.emplace_back(Vector2{ t, y1 });
			//	pointsAccurate.emplace_back(Vector2{ t, y2 });
			//	t += h;
			//	real tempy1 = y1;
			//	real tempy2 = y2;
			//	yp = y1 + h * dy1(tempy1, tempy2);
			//	yc = y1 + h * dy1(yp, tempy2);
			//	y1 = (yp + yc) / 2.0f;
			//	yp = y2 + h * dy2(tempy1, tempy2);
			//	yc = y2 + h * dy2(tempy1, yp);
			//	y2 = (yp + yc) / 2.0f;
			//}
			//RendererQtImpl::renderPoints(painter, m_camera, points, enhancedEuler);
			//RendererQtImpl::renderPoints(painter, m_camera, pointsAccurate, accurate);

		}
	private:

	};
}
#endif