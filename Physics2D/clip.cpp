#include "clip.h"
namespace Physics2D
{
	Container::Vector<Vector2> ContactGenerator::dumpVertices(const ShapePrimitive& primitive)
	{
		Container::Vector<Vector2> vertices;
		switch (primitive.shape->type())
		{
		case Shape::Type::Capsule:
		{
			const Capsule* capsule = static_cast<Capsule*>(primitive.shape);
			vertices = capsule->boxVertices();
			break;
		}
		case Shape::Type::Polygon:
		{
			const Polygon* polygon = static_cast<Polygon*>(primitive.shape);
			vertices = polygon->vertices();
			break;
		}
		case Shape::Type::Edge:
		{
			const Edge* edge = static_cast<Edge*>(primitive.shape);
			vertices.emplace_back(edge->startPoint());
			vertices.emplace_back(edge->endPoint());
			break;
		}
		case Shape::Type::Sector:
		{
			const Sector* sector = static_cast<Sector*>(primitive.shape);
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

	ContactGenerator::ClipEdge ContactGenerator::findClipEdge(const Container::Vector<Vector2>& vertices, size_t index, const Vector2& normal)
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

	ContactGenerator::ClipEdge ContactGenerator::dumpClipEdge(const ShapePrimitive& shape, const Container::Vector<Vector2>& vertices, const Vector2& normal)
	{
		ClipEdge edge;
		if (vertices.size() == 2)
		{
			edge.p1 = vertices[0];
			edge.p2 = vertices[1];
			if(shape.shape->type() == Shape::Type::Edge)
				edge.normal = static_cast<Edge*>(shape.shape)->normal();
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
		Container::Vector<Vector2> verticesA = dumpVertices(shapeA);
		Container::Vector<Vector2> verticesB = dumpVertices(shapeB);
		ClipEdge edgeA = dumpClipEdge(shapeA, verticesA, -normal);
		ClipEdge edgeB = dumpClipEdge(shapeB, verticesB, normal);
		return std::make_pair(edgeA, edgeB);
	}

	Container::Vector<PointPair> ContactGenerator::clip(const ClipEdge& clipEdgeA, const ClipEdge& clipEdgeB, const Vector2& normal)
	{
		Container::Vector<PointPair> result;
		if (clipEdgeA.isEmpty() || clipEdgeB.isEmpty())
			return result;
		//find reference edge
		real d1 = Vector2(clipEdgeA.p1 - clipEdgeA.p2).dot(normal);
		real d2 = Vector2(clipEdgeB.p1 - clipEdgeB.p2).dot(normal);
		ClipEdge referenceEdge = clipEdgeA;
		ClipEdge incidentEdge = clipEdgeB;
		bool swap = false;
		if(std::fabs(d1) > std::fabs(d2))
		{
			//edge B is reference edge
			referenceEdge = clipEdgeB;
			incidentEdge = clipEdgeA;
			swap = true;
		}

		//1. clip left region
		Vector2 u = (referenceEdge.p2 - referenceEdge.p1).normal();
		Vector2 refAnchor1 = u.perpendicular() + referenceEdge.p1;
		if(!GeometryAlgorithm2D::isPointOnSameSide(referenceEdge.p1, refAnchor1, referenceEdge.p2, incidentEdge.p1))
			incidentEdge.p1 = GeometryAlgorithm2D::lineIntersection(referenceEdge.p1, refAnchor1, incidentEdge.p1, incidentEdge.p2);
		if (!GeometryAlgorithm2D::isPointOnSameSide(referenceEdge.p1, refAnchor1, referenceEdge.p2, incidentEdge.p2))
			incidentEdge.p2 = GeometryAlgorithm2D::lineIntersection(referenceEdge.p1, refAnchor1, incidentEdge.p1, incidentEdge.p2);


		//2. clip right region
		u.negate();
		Vector2 refAnchor2 = u.perpendicular() + referenceEdge.p2;
		if (!GeometryAlgorithm2D::isPointOnSameSide(referenceEdge.p2, refAnchor2, referenceEdge.p1, incidentEdge.p1))
			incidentEdge.p1 = GeometryAlgorithm2D::lineIntersection(referenceEdge.p2, refAnchor2, incidentEdge.p1, incidentEdge.p2);
		if (!GeometryAlgorithm2D::isPointOnSameSide(referenceEdge.p2, refAnchor2, referenceEdge.p1, incidentEdge.p2))
			incidentEdge.p2 = GeometryAlgorithm2D::lineIntersection(referenceEdge.p2, refAnchor2, incidentEdge.p1, incidentEdge.p2);


		//3. clip normal region
		Vector2 refAnchor3 = (referenceEdge.p2 + referenceEdge.p1) / 2.0f + referenceEdge.normal;
		
		bool p1OnClipArea = GeometryAlgorithm2D::isPointOnSameSide(referenceEdge.p1, referenceEdge.p2, refAnchor3, incidentEdge.p1);
		bool p2OnClipArea = GeometryAlgorithm2D::isPointOnSameSide(referenceEdge.p1, referenceEdge.p2, refAnchor3, incidentEdge.p2);

		if (!(p1OnClipArea && p2OnClipArea))
			return result;

		if(p1OnClipArea && !p2OnClipArea)//p1 inside, p2 outside
			incidentEdge.p2 = GeometryAlgorithm2D::lineIntersection(referenceEdge.p1, referenceEdge.p2, incidentEdge.p1, incidentEdge.p2);
		
		else if(!p1OnClipArea && p2OnClipArea)//p1 outside, p2 inside
			incidentEdge.p1 = GeometryAlgorithm2D::lineIntersection(referenceEdge.p1, referenceEdge.p2, incidentEdge.p1, incidentEdge.p2);


		//p1 and p2 are inside, clip nothing, just go to project
		//4. project to reference edge
		Vector2 pp1 = GeometryAlgorithm2D::pointToLineSegment(referenceEdge.p1, referenceEdge.p2, incidentEdge.p1);
		Vector2 pp2 = GeometryAlgorithm2D::pointToLineSegment(referenceEdge.p1, referenceEdge.p2, incidentEdge.p2);
		result.reserve(2);
		PointPair pair1, pair2;
		if(!swap)
		{
			pair1.pointA = pp1;
			pair1.pointB = incidentEdge.p1;
			pair2.pointA = pp2;
			pair2.pointB = incidentEdge.p2;
		}
		else
		{
			pair1.pointA = incidentEdge.p1;
			pair1.pointB = pp1;
			pair2.pointA = incidentEdge.p2;
			pair2.pointB = pp2;
		}
		result.emplace_back(pair1);
		result.emplace_back(pair2);
		return result;
	}



}
