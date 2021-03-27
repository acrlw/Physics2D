#pragma once

#include "include/math/math.h"
#include "include/common/common.h"
#include "include/collision/contact.h"
#include "include/dynamics/shape.h"
namespace Physics2D
{

	struct Minkowski
	{
		Vector2 pointA;
		Vector2 pointB;
		Vector2 result;
	};
	struct ShapePrimitive
	{
		Shape * shape;
		Vector2 position;
		number angle = 0;
	};
	struct Simplex
	{
		std::vector<Minkowski> vertices;
		static inline bool isContainOrigin(const Simplex& simplex)
		{
			switch (simplex.vertices.size())
			{
            case 3:
	            {
					number a = 0, b = 0, c = 0;
					Vector2 origin;
					Vector2 oa = origin - simplex.vertices[0].result;
					Vector2 ob = origin - simplex.vertices[1].result;
					Vector2 oc = origin - simplex.vertices[2].result;

					a = Vector2::crossProduct(a, ob);
					b = Vector2::crossProduct(ob, oc);
					c = Vector2::crossProduct(oc, oa);

					if ((a <= 0 && b <= 0 && c <= 0) ||
						(a >= 0 && b >= 0 && c >= 0))
						return true;
					return false;
	            }
            case 2:
	            {
					Vector2 origin;
					Vector2 oa = origin - simplex.vertices[0].result;
					Vector2 ob = origin - simplex.vertices[1].result;
					return Vector2::crossProduct(oa, ob) == 0;
	            }
			default:
                return false;
			}
		}
		inline bool insert(const Simplex& edge, const Minkowski& vertex)
		{
			size_t targetIndex = -1;
			for (size_t i = 0; i < vertices.size() - 1; i++)
				if (vertices[i].result == edge.vertices[0].result)
					targetIndex = i;
				
			vertices.insert(vertices.begin() + targetIndex + 1, vertex);
			return targetIndex == -1;
		}
	};



	class GJK
	{
	public:
		static ContactInfo test(const ShapePrimitive& shape_A, const ShapePrimitive& shapeB)
		{
			ContactInfo result;
			Simplex simplex;

			
		}
		
		static Minkowski support()
		{
			
		}
		static Vector2 calculateDirection(const ShapePrimitive& shape, const Vector2& direction)
		{
			Vector2 target;
			Rotation2 rot = Rotation2(-1 * shape.angle);
			Vector2 rot_dir = rot.multiply(direction);
			switch (shape.shape->type())
			{
				case Shape::Type::Polygon:
				{
					number max = 0.0f;
					auto polygon = dynamic_cast<const Polygon*>(shape.shape);
					for (auto vertex : polygon->vertices())
					{
						number result = Vector2::dotProduct(vertex, rot_dir);
						
						if (max == 0.0f)
							continue;
						
						max = max < result ? result : max;
						target = vertex;
					}
					break;
				}
				case Shape::Type::Circle:
				{
					auto circle = dynamic_cast<const Circle*>(shape.shape);
					target = rot_dir.normalize() * circle->radius();
					break;
				}
				case Shape::Type::Ellipse:
				{
					auto ellipse = dynamic_cast<const Ellipse*>(shape.shape);
					number a = ellipse->A();
					number b = ellipse->A();
					if(rot_dir.x == 0.0f)
					{
						int sgn = direction.y < 0 ? -1 : 1;
						target.set(0, sgn * b);
					}
					else if(rot_dir.y == 0.0f)
					{
						int sgn = direction.x < 0 ? -1 : 1;
						target.set(sgn * a,0);
					}
					else
					{
						float k = rot_dir.y / rot_dir.x;
						//line offset constant d
						float a2 = pow(a, 2);
						float b2 = pow(b, 2);
						float k2 = pow(k, 2);
						float d = sqrt((a2 + b2 * k2) / k2);
						float x1, y1;
						if (Vector2::dotProduct(Vector2(0, d), rot_dir) < 0)
							d = d * -1;
						x1 = k * d - (b2 * k2 * k * d) / (a2 + b2 * k2);
						y1 = (b2 * k2 * d) / (a2 + b2 * k2);
						target.set(x1, y1);
						//rotate the final vector
						rot.setAngle(shape.angle);
					}
					break;
				}
				case Shape::Type::Edge:
				{
					auto edge = dynamic_cast<const Edge*>(shape.shape);
					number dot1, dot2;
					dot1 = Vector2::dotProduct(edge->startPoint(), direction);
					dot2 = Vector2::dotProduct(edge->endPoint(), direction);
					target = dot1 > dot2 ? edge->startPoint() : edge->endPoint();
					break;
				}
				default: 
				break;
			}
			target = rot * target;
			return target;
		}
	private:
		Vector2 m_minPenetrVec;
	};
}