#include "include/collision/algorithm/clip.h"
namespace Physics2D
{
	std::vector<Vector2> ContactGenerator::dumpVertices(const ShapePrimitive& primitive)
	{
		std::vector<Vector2> vertices;
		switch (primitive.shape->type())
		{
		case Shape::Type::Capsule:
		{
			const Capsule* capsule = dynamic_cast<Capsule*>(primitive.shape.get());
			vertices = capsule->boxVertices();
			break;
		}
		case Shape::Type::Polygon:
		{
			const Polygon* polygon = dynamic_cast<Polygon*>(primitive.shape.get());
			vertices = polygon->vertices();
			break;
		}
		case Shape::Type::Edge:
		{
			const Edge* edge = dynamic_cast<Edge*>(primitive.shape.get());
			vertices.emplace_back(edge->startPoint());
			vertices.emplace_back(edge->endPoint());
			break;
		}
		case Shape::Type::Sector:
		{
			const Sector* sector = dynamic_cast<Sector*>(primitive.shape.get());
			vertices = sector->vertices();
			break;
		}
		default:
			break;
		}
		for (auto& elem : vertices)
			elem = primitive.translate(elem);
		return vertices;
	}

	ContactGenerator::ClipEdge ContactGenerator::findClipEdge(const std::vector<Vector2>& vertices, size_t index, const Vector2& normal)
	{
		ClipEdge edge1, edge2;
		edge1.p2 = vertices[index];
		edge2.p1 = vertices[index];
		if (index == 0)
		{
			edge1.p1 = vertices[vertices.size() - 2];
			edge2.p2 = vertices[index + 1];
		}
		else if (index == vertices.size() - 1)
		{
			edge1.p1 = vertices[index - 1];
			edge2.p2 = vertices[1];
		}
		else
		{
			edge1.p1 = vertices[index - 1];
			edge2.p2 = vertices[index + 1];
		}
		//compare which is closest to normal
		ClipEdge finalEdge;
		if(std::fabs((edge1.p2 - edge1.p1).dot(normal)) >= std::fabs((edge2.p2 - edge2.p1).dot(normal)))
		{
			finalEdge = edge2;
			Vector2 p = (edge2.p2 - edge2.p1).normal().perpendicular();
			finalEdge.normal = GeometryAlgorithm2D::isPointOnSameSide(edge2.p1, edge2.p2, edge1.p1, edge2.p1 + p) ? p : -p;
		}
		else
		{
			finalEdge = edge1;
			Vector2 p = (edge1.p2 - edge1.p1).normal().perpendicular();
			finalEdge.normal = GeometryAlgorithm2D::isPointOnSameSide(edge1.p1, edge1.p2, edge2.p2, edge1.p1 + p) ? p : -p;
		}
		return finalEdge;
	}

	ContactGenerator::ClipEdge ContactGenerator::dumpClipEdge(const ShapePrimitive& shape, const std::vector<Vector2>& vertices, const Vector2& normal)
	{
		ClipEdge edge;
		if (vertices.size() == 2)
		{
			edge.p1 = vertices[0];
			edge.p2 = vertices[1];
			if(shape.shape->type() == Shape::Type::Edge)
				edge.normal = dynamic_cast<Edge*>(shape.shape.get())->normal();
		}
		else
		{
			auto [support, index] = GJK::findFarthestPoint(vertices, normal);
			edge = findClipEdge(vertices, index, normal);
		}
		return edge;
	}

	std::pair<ContactGenerator::ClipEdge, ContactGenerator::ClipEdge> ContactGenerator::recognize(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& normal)
	{
		auto typeA = shapeA.shape->type();
		auto typeB = shapeB.shape->type();
		if (typeA == Shape::Type::Point || typeA == Shape::Type::Circle || typeA == Shape::Type::Ellipse
			|| typeB == Shape::Type::Point || typeB == Shape::Type::Circle || typeB == Shape::Type::Ellipse)
			return std::make_pair(ClipEdge(), ClipEdge());
		//normal: B -> A
		std::vector<Vector2> verticesA = dumpVertices(shapeA);
		std::vector<Vector2> verticesB = dumpVertices(shapeB);
		ClipEdge edgeA = dumpClipEdge(shapeA, verticesA, -normal);
		ClipEdge edgeB = dumpClipEdge(shapeB, verticesB, normal);
		return std::make_pair(edgeA, edgeB);
	}

	std::vector<PointPair> ContactGenerator::clip(const ClipEdge& clipEdgeA, const ClipEdge& clipEdgeB, const Vector2& normal)
	{
		std::vector<PointPair> result;

		return result;
	}

	std::vector<Vector2> Clipper::sutherlandHodgmentPolygonClipping(const std::vector<Vector2>& polygon, const std::vector<Vector2>& clipRegion)
	{
		std::vector<Vector2> result = polygon;

		for (size_t i = 0; i < clipRegion.size() - 1; i++)
		{
			Vector2 clipPoint1 = clipRegion[i];
			Vector2 clipPoint2 = clipRegion[i + 1];
			Vector2 clipDirectionPoint = i + 2 == clipRegion.size() ? clipRegion[1] : clipRegion[i + 2];
			std::vector<int8_t> testResults;
			testResults.reserve(polygon.size());

			for (size_t j = 0; j < result.size(); j++)
			{
				bool res = GeometryAlgorithm2D::isPointOnSameSide(clipPoint1, clipPoint2, clipDirectionPoint, result[j]);
				testResults.emplace_back(res ? 1 : -1);
			}
			std::vector<Vector2> newPolygon;
			newPolygon.reserve(result.size());
			//test result:
			//1: inside, -1: outside
			//2: processed, 3: need to clear
			for (size_t j = 1; j < testResults.size(); j++)
			{
				bool lastInside = testResults[j - 1] == 1 ? true : false;
				bool currentInside = testResults[j] == 1 ? true : false;
				//last inside and current outside
				if (lastInside && !currentInside)
				{
					//push last point
					newPolygon.emplace_back(result[j - 1]);
					//push intersection point
					Vector2 p = GeometryAlgorithm2D::lineIntersection(clipPoint1, clipPoint2, result[j - 1], result[j]);
					newPolygon.emplace_back(p);
				}
				//last outside and current inside
				if (!lastInside && currentInside)
				{
					//push intersection point first
					Vector2 p = GeometryAlgorithm2D::lineIntersection(clipPoint1, clipPoint2, result[j - 1], result[j]);
					newPolygon.emplace_back(p);
				}
				//last outside and current outside
				if (!lastInside && !currentInside)
				{
					//do nothing
				}
				if (lastInside && currentInside)
				{
					//push last vertex
					newPolygon.emplace_back(result[j - 1]);
				}
			}
			result = newPolygon;
			result.emplace_back(result[0]);
		}
		return result;
	}

}
