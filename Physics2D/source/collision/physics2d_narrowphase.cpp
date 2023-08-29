#include "physics2d_narrowphase.h"

#include <iostream>

namespace Physics2D
{
	Simplex Narrowphase::gjk(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration)
	{
		Simplex simplex;

		Vector2 direction = shapeB.transform.position - shapeA.transform.position;

		if (direction.fuzzyEqual({0, 0}))
			direction.set(1, 1);
		//first
		SimplexVertex vertex = support(shapeA, shapeB, direction);
		simplex.addSimplexVertex(vertex);
		//second
		direction.negate();
		vertex = support(shapeA, shapeB, direction);
		simplex.addSimplexVertex(vertex);

		//check 1d simplex(line segment) across origin
		//WARN: this can be used to check collision but not friendly with EPA, here adding perturbation to avoid 1d simplex

		//try to reconfigure simplex to avoid 1d simplex cross origin
		if (simplex.containsOrigin())
		{
			const bool result = perturbSimplex(simplex, shapeA, shapeB, direction);
			if (!result)
				assert(false && "Cannot reconstruct simplex.");
		}

		//third
		for (Index iter = 0; iter <= iteration; ++iter)
		{
			//default closest edge is index 0 and index 1
			direction = findDirectionByEdge(simplex.vertices[0], simplex.vertices[1], true);
			vertex = support(shapeA, shapeB, direction);

			//find repeated vertex
			if (simplex.contains(vertex))
				break;

			//vertex does not pass origin
			if (vertex.result.dot(direction) <= 0)
				break;

			simplex.addSimplexVertex(vertex);

			reconstructSimplexByVoronoi(simplex);
			if (simplex.isContainOrigin)
				return simplex;

			simplex.removeEnd();
		}
		return simplex;
	}

	CollisionInfo Narrowphase::epa(const Simplex& simplex, const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                               const size_t& iteration, const real& epsilon)
	{
		//return 1d simplex with edge closest to origin
		CollisionInfo info;
		info.simplex = simplex;
		info.simplex.removeEnd();

		//initiate polytope

		//[Debug]
		std::list<SimplexVertexWithOriginDistance>& polytope = info.polytope;
		//std::list<SimplexVertexWithOriginDistance> polytope;

		buildPolytopeFromSimplex(polytope, simplex);

		auto iterStart = polytope.begin();
		auto iterEnd = polytope.end();
		auto iterTemp = polytope.begin();

		for (Index iter = 0; iter < iteration; ++iter)
		{
			//indices of closest edge are set to 0 and 1
			const Vector2 direction = findDirectionByEdge(info.simplex.vertices[0], info.simplex.vertices[1], false);

			const SimplexVertex vertex = support(shapeA, shapeB, direction);

			//cannot find any new vertex
			if (info.simplex.contains(vertex))
				break;

			//convex test, make sure polytope is always convex

			auto itA = iterStart;

			auto itB = itA;
			polytopeIterNext(itB, polytope);

			auto itC = itB;
			polytopeIterNext(itC, polytope);

			const Vector2 ab = itB->vertex.result - itA->vertex.result;
			const Vector2 bc = itC->vertex.result - itB->vertex.result;
			const real res1 = Vector2::crossProduct(ab, bc);

			const Vector2 an = vertex.result - itA->vertex.result;
			const Vector2 nb = itB->vertex.result - vertex.result;
			const real res2 = Vector2::crossProduct(an, nb);

			const real res3 = Vector2::crossProduct(nb, bc);

			const bool validConvexity = Math::sameSignStrict(res1, res2, res3);

			if (!validConvexity) //invalid vertex, just break
				break;

			//then insert new vertex

			SimplexVertexWithOriginDistance pair;
			pair.vertex = vertex;
			const Vector2 t1 = GeometryAlgorithm2D::pointToLineSegment(itA->vertex.result, vertex.result, {0, 0});
			const real dist1 = t1.lengthSquare();
			const Vector2 t2 = GeometryAlgorithm2D::pointToLineSegment(vertex.result, itB->vertex.result, {0, 0});
			const real dist2 = t2.lengthSquare();

			itA->distance = dist1;
			pair.distance = dist2;
			polytope.insert(itB, pair);

			//set to begin
			iterTemp = iterStart;

			//find shortest distance and set iterStart
			real minDistance = Constant::Max;
			auto iterTarget = iterStart;
			while (true)
			{
				if (iterTemp->distance < minDistance)
				{
					minDistance = iterTemp->distance;
					iterTarget = iterTemp;
				}
				polytopeIterNext(iterTemp, polytope);
				if (iterTemp == iterStart)
					break;
			}
			iterStart = iterTarget;
			iterEnd = iterTarget;
			polytopeIterPrev(iterEnd, polytope);

			//set to begin
			iterTemp = iterStart;
			polytopeIterNext(iterTemp, polytope);
			//reset simplex
			info.simplex.vertices[0] = iterStart->vertex;
			info.simplex.vertices[1] = iterTemp->vertex;


		}

		const Vector2 temp = -GeometryAlgorithm2D::pointToLineSegment(info.simplex.vertices[0].result,
		                                                              info.simplex.vertices[1].result
		                                                              , {0, 0});

		info.penetration = temp.length();
		//assert(!realEqual(info.penetration, 0));
		info.normal.clear();
		//penetration is close to zero, just return
		if (!realEqual(info.penetration, 0))
			info.normal = temp / info.penetration;

		return info;
	}

	SimplexVertex Narrowphase::support(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                   const Vector2& direction)
	{
		SimplexVertex vertex;
		std::tie(vertex.point[0], vertex.index[0]) = findFurthestPoint(shapeA, direction);
		std::tie(vertex.point[1], vertex.index[1]) = findFurthestPoint(shapeB, direction.negative());
		vertex.result = vertex.point[0] - vertex.point[1];
		return vertex;
	}

	std::pair<Vector2, Index> Narrowphase::findFurthestPoint(const ShapePrimitive& shape, const Vector2& direction)
	{
		Vector2 target;
		Matrix2x2 rot(-shape.transform.rotation);
		Vector2 rot_dir = rot.multiply(direction);
		Index finalIndex = UINT32_MAX;
		switch (shape.shape->type())
		{
		case Shape::Type::Polygon:
			{
				auto polygon = static_cast<const Polygon*>(shape.shape);
				std::tie(target, finalIndex) = findFurthestPoint(polygon->vertices(), rot_dir);
				break;
			}
		case Shape::Type::Circle:
			{
				auto circle = static_cast<const Circle*>(shape.shape);
				return std::make_pair(direction.normal() * circle->radius() + shape.transform.position, finalIndex);
			}
		case Shape::Type::Ellipse:
			{
				auto ellipse = static_cast<const Ellipse*>(shape.shape);
				target = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), rot_dir);
				break;
			}
		case Shape::Type::Edge:
			{
				auto edge = static_cast<const Edge*>(shape.shape);
				const real dot1 = Vector2::dotProduct(edge->startPoint(), direction);
				const real dot2 = Vector2::dotProduct(edge->endPoint(), direction);
				target = dot1 > dot2 ? edge->startPoint() : edge->endPoint();
				break;
			}
		case Shape::Type::Capsule:
			{
				auto capsule = static_cast<const Capsule*>(shape.shape);
				target = GeometryAlgorithm2D::calculateCapsuleProjectionPoint(
					capsule->halfWidth(), capsule->halfHeight(), rot_dir);
				finalIndex = 0;
				const Vector2 test(Math::abs(target.x), Math::abs(target.y));
				const Vector2 topRight = capsule->topRight();
				if (test.equal(topRight))
					finalIndex = 1;
				break;
			}
		}
		rot.set(shape.transform.rotation);
		target = rot.multiply(target);
		target += shape.transform.position;
		return std::make_pair(target, finalIndex);
	}

	Vector2 Narrowphase::findDirectionByEdge(const SimplexVertex& v1, const SimplexVertex& v2, bool pointToOrigin)

	{
		const Vector2 p1 = v1.result;
		const Vector2 p2 = v2.result;
		const Vector2 ao = p1.negative();
		const Vector2 ab = p2 - p1;
		Vector2 perpendicularOfAB = ab.perpendicular();
		if ((Vector2::dotProduct(ao, perpendicularOfAB) < 0 && pointToOrigin) || (
			Vector2::dotProduct(ao, perpendicularOfAB) > 0 && !pointToOrigin))
			perpendicularOfAB.negate();
		return perpendicularOfAB;
	}

	std::pair<Vector2, Index> Narrowphase::findFurthestPoint(const Container::Vector<Vector2>& vertices,
	                                                         const Vector2& direction)
	{
		real max = Constant::NegativeMin;
		Vector2 target;
		Index index = 0;
		for (Index i = 0; i < vertices.size(); i++)
		{
			real result = Vector2::dotProduct(vertices[i], direction);
			if (max < result)
			{
				max = result;
				target = vertices[i];
				index = i;
			}
		}
		return std::make_pair(target, index);
	}

	ContactPair Narrowphase::generateContacts(const ShapePrimitive& shapeA,
	                                          const ShapePrimitive& shapeB, CollisionInfo& info)
	{
		ContactPair pair;
		Shape::Type typeA = shapeA.shape->type();
		Shape::Type typeB = shapeB.shape->type();

		ShapePrimitive realShapeA = shapeA;
		ShapePrimitive realShapeB = shapeB;
		Index idxA = 0;
		Index idxB = 1;

		const bool isSwap = typeA > typeB;

		if (isSwap)
		{
			std::swap(realShapeA, realShapeB);
			std::swap(typeA, typeB);
			std::swap(idxA, idxB);
			//temporarily negate
			info.normal.negate();
		}

		//find feature
		const Feature featureA = findFeatures(info.simplex, info.normal, realShapeA, idxA);
		const Feature featureB = findFeatures(info.simplex, info.normal, realShapeB, idxB);
		auto idA = std::pair{ featureA.index[0], featureA.index[1] };
		pair.ids[0] = reinterpret_cast<uint64_t&>(idA);

		auto idB = std::pair{ featureB.index[0], featureB.index[1] };
		pair.ids[1] = reinterpret_cast<uint64_t&>(idB);

		if (typeA == Shape::Type::Polygon)
		{
			switch (typeB)
			{
			case Shape::Type::Polygon:
				pair = clipPolygonPolygon(realShapeA, realShapeB, featureA, featureB, info);
				break;
			case Shape::Type::Edge:
				pair = clipPolygonEdge(realShapeA, realShapeB, featureA, featureB, info);
				break;
			case Shape::Type::Capsule:
				pair = clipPolygonCapsule(realShapeA, realShapeB, featureA, featureB, info);
				break;
			case Shape::Type::Circle:
			case Shape::Type::Ellipse:
				pair = clipPolygonRound(realShapeA, realShapeB, featureA, featureB, info);
				break;
			}
		}
		else if (typeA == Shape::Type::Edge)
		{
			switch (typeB)
			{
			case Shape::Type::Edge:
				assert(false && "Not support edge and edge.");
				break;
			case Shape::Type::Capsule:
				pair = clipEdgeCapsule(realShapeA, realShapeB, featureA, featureB, info);
				break;
			case Shape::Type::Circle:
			case Shape::Type::Ellipse:
				pair = clipEdgeRound(realShapeA, realShapeB, featureA, featureB, info);
				break;
			}
		}
		else if (typeA == Shape::Type::Capsule)
		{
			switch (typeB)
			{
			case Shape::Type::Capsule:
				pair = clipCapsuleCapsule(realShapeA, realShapeB, featureA, featureB, info);
				break;
			case Shape::Type::Circle:
			case Shape::Type::Ellipse:
				pair = clipCapsuleRound(realShapeA, realShapeB, featureA, featureB, info);
				break;
			}
		}
		else //round round case
		{
			pair = clipRoundRound(realShapeA, realShapeB, featureA, featureB, info);
			return pair;
		}

		if (isSwap)
		{
			std::swap(pair.points[0], pair.points[1]);
			std::swap(pair.points[2], pair.points[3]);
			std::swap(pair.ids[0], pair.ids[1]);
			//restore normal
			info.normal.negate();
		}
		return pair;
	}

	CollisionInfo Narrowphase::gjkDistance(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                       const size_t& iteration)
	{
		VertexPair result;
		CollisionInfo info;

		Vector2 direction = shapeB.transform.position - shapeA.transform.position;

		if (direction.fuzzyEqual({0, 0}))
			direction.set(1, 1);
		//first
		SimplexVertex vertex = support(shapeA, shapeB, direction);
		info.simplex.addSimplexVertex(vertex);
		//second
		direction.negate();
		vertex = support(shapeA, shapeB, direction);
		info.simplex.addSimplexVertex(vertex);
		//third
		direction = direction.perpendicular();
		vertex = support(shapeA, shapeB, direction);
		info.simplex.addSimplexVertex(vertex);

		reconstructSimplexByVoronoi(info.simplex);
		info.originalSimplex = info.simplex;
		//[DEBUG]
		std::list<SimplexVertexWithOriginDistance>& polytope = info.polytope;
		//std::list<SimplexVertexWithOriginDistance> polytope;

		buildPolytopeFromSimplex(polytope, info.simplex);

		auto iterStart = polytope.begin();
		auto iterEnd = polytope.end();
		auto iterTemp = polytope.begin();

		int errorCount = 0;

		auto reindexSimplex = [&info, &polytope, &iterStart, &iterEnd, &iterTemp]
		{
			std::swap(info.simplex.vertices[1], info.simplex.vertices[2]);
			std::swap(info.simplex.vertices[0], info.simplex.vertices[1]);
			polytope.clear();
			buildPolytopeFromSimplex(polytope, info.simplex);
			iterStart = polytope.begin();
			iterEnd = polytope.end();
			iterTemp = polytope.begin();
		};

		int sameDistCount = 0;

		for (Index iter = 0; iter < iteration; ++iter)
		{
			//indices of closest edge are set to 0 and 1
			direction = findDirectionByEdge(info.simplex.vertices[0], info.simplex.vertices[1], true);

			vertex = support(shapeA, shapeB, direction);

			//cannot find any new vertex
			if (info.simplex.contains(vertex))
			{
				if(sameDistCount == 1)
				{
					//check edge case
					polytopeIterNext(iterStart, polytope);
					iterEnd = iterStart;
					polytopeIterPrev(iterEnd, polytope);
					iterTemp = iterStart;
					polytopeIterNext(iterTemp, polytope);

					info.simplex.vertices[0] = iterStart->vertex;
					info.simplex.vertices[1] = iterTemp->vertex;
					iter--;
					//do not process anymore
					sameDistCount = -1;
					continue;
				}
				if (polytope.size() >= 4) //polytope has been expanded, terminate the loop
					break;

				if (errorCount == 3) //fail to rewind simplex, terminate
					break;

				reindexSimplex();

				iter--;

				errorCount++;

				continue;
			}
			//convex test, make sure polytope is always convex

			auto itA = iterStart;

			auto itB = itA;
			polytopeIterNext(itB, polytope);

			auto itC = itB;
			polytopeIterNext(itC, polytope);

			const Vector2 ab = itB->vertex.result - itA->vertex.result;
			const Vector2 bc = itC->vertex.result - itB->vertex.result;
			const real res1 = Vector2::crossProduct(ab, bc);

			const Vector2 an = vertex.result - itA->vertex.result;
			const Vector2 nb = itB->vertex.result - vertex.result;
			const real res2 = Vector2::crossProduct(an, nb);

			const real res3 = Vector2::crossProduct(nb, bc);

			const bool validConvexity = Math::sameSignStrict(res1, res2, res3);

			if (!validConvexity) //invalid vertex
			{
				if (polytope.size() >= 4) //if polytope is expanded, terminate the loop
					break;

				if (errorCount == 3) //fail to rewind simplex, terminate
					break;

				//try to rewind
				reindexSimplex();
				errorCount++;
				iter--;
				continue;
			}

			//then insert new vertex

			SimplexVertexWithOriginDistance pair;
			pair.vertex = vertex;
			const Vector2 t1 = GeometryAlgorithm2D::pointToLineSegment(itA->vertex.result, vertex.result, {0, 0});
			const real dist1 = t1.lengthSquare();
			const Vector2 t2 = GeometryAlgorithm2D::pointToLineSegment(vertex.result, itB->vertex.result, {0, 0});
			const real dist2 = t2.lengthSquare();

			itA->distance = dist1;
			pair.distance = dist2;
			polytope.insert(itB, pair);
			//TODO: if dist1 == dist2, and dist1 cannot be extended and dist2 can be extended.
			sameDistCount = realEqual(dist1, dist2) ? sameDistCount + 1 : sameDistCount;

			//set to begin
			iterTemp = iterStart;

			//find shortest distance and set iterStart
			real minDistance = Constant::Max;
			auto iterTarget = iterStart;
			while (true)
			{
				if (iterTemp->distance < minDistance)
				{
					minDistance = iterTemp->distance;
					iterTarget = iterTemp;
				}
				polytopeIterNext(iterTemp, polytope);
				if (iterTemp == iterStart)
					break;
			}
			iterStart = iterTarget;
			iterEnd = iterTarget;
			polytopeIterPrev(iterEnd, polytope);

			//set to begin
			iterTemp = iterStart;
			polytopeIterNext(iterTemp, polytope);
			//reset simplex
			info.simplex.vertices[0] = iterStart->vertex;
			info.simplex.vertices[1] = iterTemp->vertex;
			errorCount = 0;
		}
		auto idxStart = std::distance(polytope.begin(), iterStart);
		polytopeIterNext(iterStart, polytope);
		auto idxNext = std::distance(polytope.begin(), iterStart);
		info.simplex.removeEnd();
		//Convex combination for calculating distance points
		//https://dyn4j.org/2010/04/gjk-distance-closest-points/

		Vector2& A_0 = info.simplex.vertices[0].point[0];
		Vector2& B_0 = info.simplex.vertices[0].point[1];
		Vector2& A_1 = info.simplex.vertices[1].point[0];
		Vector2& B_1 = info.simplex.vertices[1].point[1];

		Vector2 M_0 = info.simplex.vertices[0].result;
		Vector2 M_1 = info.simplex.vertices[1].result;

		Vector2 M0_M1 = M_1 - M_0;
		real M0_O_proj = -M_0.dot(M0_M1);
		real M0_M1_proj = M0_M1.dot(M0_M1);
		real v = M0_O_proj / M0_M1_proj;
		real u = 1 - v;

		result.pointA.set(u * A_0 + v * A_1);
		result.pointB.set(u * B_0 + v * B_1);

		if(M0_M1.fuzzyEqual({0, 0}) || v < 0)
		{
			result.pointA.set(A_0);
			result.pointB.set(B_0);
		}
		else if (u < 0)
		{
			result.pointA.set(A_1);
			result.pointB.set(B_1);
		}
		info.pair = result;

		return info;
	}

	void Narrowphase::reconstructSimplexByVoronoi(Simplex& simplex)
	{
		//use barycentric coordinates to check contains origin and find closest edge
		const SimplexVertex va = simplex.vertices[0];
		const SimplexVertex vb = simplex.vertices[1];
		const SimplexVertex vc = simplex.vertices[2];
		const Vector2 a = va.result;
		const Vector2 b = vb.result;
		const Vector2 c = vc.result;
		const Vector2 ab = b - a;
		const Vector2 ac = c - a;
		const Vector2 bc = c - b;
		const real ab_length = ab.length();
		const real ac_length = ac.length();
		const real bc_length = bc.length();

		const real u_ac = -a.dot(ac.normal()) / ac_length;
		const real u_bc = -b.dot(bc.normal()) / bc_length;

		const real v_ac = 1 - u_ac;
		const real v_bc = 1 - u_bc;
		/*
		 * [Ax Bx Cx][u] = [0]
		 * [Ay By Cy][v] = [0]
		 * [ 1  1  1][w] = [1]
		 * solve for u,v,w
		 */
		const real det = a.y * b.x - a.x * b.y + a.x * c.y - a.y * c.x + b.y * c.x - c.y * b.x;
		assert(det != 0.0f);
		const real u = (b.y * c.x - c.y * b.x) / det;
		const real v = (c.y * a.x - a.y * c.x) / det;
		const real w = 1 - u - v;


		if (u_ac > 0 && v_ac > 0 && v <= 0)
			std::swap(simplex.vertices[1], simplex.vertices[2]);
		else if (u_bc > 0 && v_bc > 0 && u <= 0)
			std::swap(simplex.vertices[0], simplex.vertices[2]);
		else if (u > 0 && v > 0 && w > 0)
		{
			//in region ABC, origin is inside simplex
			simplex.isContainOrigin = true;

			//reorder simplex index, so that the closest edge is index 0 and index 1
			//h means height of triangle
			const real h_u = u / bc_length;
			const real h_v = v / ac_length;
			const real h_w = w / ab_length;
			const real min = Math::tripleMin(h_u, h_v, h_w);
			if (min == h_u) //c b a
				std::swap(simplex.vertices[0], simplex.vertices[2]);
			else if (min == h_v) //a c b
				std::swap(simplex.vertices[1], simplex.vertices[2]);
			
		}
	}

	bool Narrowphase::perturbSimplex(Simplex& simplex, const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                 const Vector2& dir)
	{
		Vector2 direction = dir;
		for (int i = 0; i < Constant::GJKRetryTimes; ++i)
		{
			direction.set(-direction.y + static_cast<real>(i), -direction.x - static_cast<real>(i));
			SimplexVertex v = support(shapeA, shapeB, direction);
			simplex.vertices[0] = v;
			direction.set(-direction.y - static_cast<real>(i) - 0.5f, -direction.x + static_cast<real>(i) + 0.5f);
			v = support(shapeA, shapeB, direction);
			simplex.vertices[1] = v;

			if (!simplex.containsOrigin())
				return true;
		}
		//can't reconstruct
		return false;
	}

	Feature Narrowphase::findFeatures(const Simplex& simplex, const Vector2& normal, const ShapePrimitive& shape,
	                                  const Index& AorB)
	{
		Feature feature;
		feature.index[0] = simplex.vertices[0].index[AorB];
		feature.index[1] = simplex.vertices[1].index[AorB];
		if (shape.shape->type() == Shape::Type::Polygon)
		{
			if (simplex.vertices[0].point[AorB] == simplex.vertices[1].point[AorB])
			{
				//same vertex case
				//find neighbor index

				auto polygon = static_cast<const Polygon*>(shape.shape);
				const Vector2 n = shape.transform.inverseRotatePoint(normal);

				const Index idxCurr = simplex.vertices[0].index[AorB];
				const size_t realSize = polygon->vertices().size();
				//TODO: change vertex convention of polygon 
				const Index idxNext = (idxCurr + 1) % realSize;
				//if idx = 0 then unsigned number overflow, so minus operation is needed to be set aside.
				const Index idxPrev = (idxCurr + realSize - 1) % realSize;


				//check most perpendicular
				const Vector2 ab = (polygon->vertices()[idxNext] - polygon->vertices()[idxCurr]).normal();
				const Vector2 ac = (polygon->vertices()[idxCurr] - polygon->vertices()[idxPrev]).normal();


				const real dot1 = Math::abs(ab.dot(n));
				const real dot2 = Math::abs(ac.dot(n));

				feature.index[0] = idxCurr;
				feature.index[1] = idxNext;

				if (dot1 > dot2)
					feature.index[1] = idxPrev;
			}
		}
		else
		{
			//copy index for all

			feature.vertex[0] = simplex.vertices[0].point[AorB];
			feature.vertex[1] = simplex.vertices[1].point[AorB];
		}
		return feature;
	}

	ContactPair Narrowphase::clipTwoEdge(const Vector2& va1, const Vector2& va2, const Vector2& vb1, const Vector2& vb2,
	                                     CollisionInfo& info)
	{
		std::array<ClipVertex, 2> incEdge;
		std::array<Vector2, 2> refEdge = {va1, va2};

		Vector2 refNormal = info.normal;

		incEdge[0].vertex = vb1;
		incEdge[1].vertex = vb2;

		const real dot1 = Math::abs((va2 - va1).dot(refNormal));
		const real dot2 = Math::abs((vb2 - vb1).dot(refNormal));
		const bool swap = dot1 > dot2;

		if (swap)
		{
			//swap
			refEdge[0] = vb1;
			refEdge[1] = vb2;
			incEdge[0].vertex = va1;
			incEdge[1].vertex = va2;
			//notice, default normal is changed.
			refNormal.negate();
		}

		return clipIncidentEdge(incEdge, refEdge, refNormal, swap);
	}

	ContactPair Narrowphase::clipIncidentEdge(std::array<ClipVertex, 2>& incEdge, std::array<Vector2, 2> refEdge,
	                                          const Vector2& normal, bool swap)
	{
		ContactPair pair;
		const Vector2 refEdgeDir = (refEdge[1] - refEdge[0]).normal();
		const Vector2 refEdgeNormal = GeometryAlgorithm2D::lineSegmentNormal(refEdge[0], refEdge[1], normal);

		//check ref1
		const bool isRef1Inc1Valid = refEdgeDir.dot(incEdge[0].vertex - refEdge[0]) >= 0;
		const bool isRef1Inc2Valid = refEdgeDir.dot(incEdge[1].vertex - refEdge[0]) >= 0;


		//assert(isRef1Inc1Valid || isRef1Inc2Valid && "Invalid features.");
		if (!isRef1Inc1Valid && !isRef1Inc2Valid)
			return pair;

		if (!isRef1Inc1Valid && isRef1Inc2Valid)
		{
			incEdge[0].isClip = true;
			incEdge[0].vertex = GeometryAlgorithm2D::rayRayIntersectionUnsafe(refEdge[0], refEdgeNormal,
			                                                                  incEdge[0].vertex,
			                                                                  incEdge[1].vertex - incEdge[0].vertex);
			incEdge[0].clipperVertex = refEdge[0];
		}
		else if (isRef1Inc1Valid && !isRef1Inc2Valid)
		{
			incEdge[1].isClip = true;
			incEdge[1].vertex = GeometryAlgorithm2D::rayRayIntersectionUnsafe(refEdge[0], refEdgeNormal,
			                                                                  incEdge[0].vertex,
			                                                                  incEdge[1].vertex - incEdge[0].vertex);
			incEdge[1].clipperVertex = refEdge[0];
		}
		//check ref2
		const Vector2 test1 = incEdge[0].vertex - refEdge[1];
		const Vector2 test2 = incEdge[1].vertex - refEdge[1];
		const real dot1 = refEdgeDir.dot(test1);
		const real dot2 = refEdgeDir.dot(test2);
		const bool isRef2Inc1Valid = refEdgeDir.dot(incEdge[0].vertex - refEdge[1]) <= 0;
		const bool isRef2Inc2Valid = refEdgeDir.dot(incEdge[1].vertex - refEdge[1]) <= 0;

		//assert(isRef2Inc1Valid || isRef2Inc2Valid && "Invalid features.");
		if (!isRef2Inc1Valid && !isRef2Inc2Valid)
			return pair;

		if (!isRef2Inc1Valid && isRef2Inc2Valid)
		{
			incEdge[0].isClip = true;
			incEdge[0].vertex = GeometryAlgorithm2D::rayRayIntersectionUnsafe(refEdge[1], refEdgeNormal,
			                                                                  incEdge[0].vertex,
			                                                                  incEdge[1].vertex - incEdge[0].vertex);
			incEdge[0].clipperVertex = refEdge[1];
		}
		else if (isRef2Inc1Valid && !isRef2Inc2Valid)
		{
			incEdge[1].isClip = true;
			incEdge[1].vertex = GeometryAlgorithm2D::rayRayIntersectionUnsafe(refEdge[1], refEdgeNormal,
			                                                                  incEdge[0].vertex,
			                                                                  incEdge[1].vertex - incEdge[0].vertex);
			incEdge[1].clipperVertex = refEdge[1];
		}

		//check ref normal region
		incEdge[0].isFinalValid = (incEdge[0].vertex - refEdge[0]).dot(refEdgeNormal) >= 0;
		incEdge[1].isFinalValid = (incEdge[1].vertex - refEdge[0]).dot(refEdgeNormal) >= 0;

		assert(incEdge[0].isFinalValid || incEdge[1].isFinalValid && "Invalid features.");

		if (incEdge[0].isFinalValid && !incEdge[1].isFinalValid)
		{
			//discard invalid, project valid point to segment
			Vector2 incContact1 = incEdge[0].vertex;
			Vector2 refContact1 = incEdge[0].clipperVertex;

			if (!incEdge[0].isClip)
			{
				incContact1 = incEdge[0].vertex;
				refContact1 = GeometryAlgorithm2D::pointToLineSegment(refEdge[0], refEdge[1], incEdge[0].vertex);
			}

			if (swap)
				std::swap(incContact1, refContact1);

			pair.addContact(refContact1, incContact1);
		}
		else if (!incEdge[0].isFinalValid && incEdge[1].isFinalValid)
		{
			//discard invalid, project valid point to segment
			Vector2 incContact2 = incEdge[1].vertex;
			Vector2 refContact2 = incEdge[1].clipperVertex;

			if (!incEdge[1].isClip)
			{
				incContact2 = incEdge[1].vertex;
				refContact2 = GeometryAlgorithm2D::pointToLineSegment(refEdge[0], refEdge[1], incEdge[1].vertex);
			}

			if (swap)
				std::swap(incContact2, refContact2);

			pair.addContact(refContact2, incContact2);
		}
		else
		{
			//both valid, continue to check isClip
			Vector2 incContact1 = incEdge[0].vertex;
			Vector2 refContact1 = incEdge[0].clipperVertex;

			Vector2 incContact2 = incEdge[1].vertex;
			Vector2 refContact2 = incEdge[1].clipperVertex;

			if (!incEdge[0].isClip)
			{
				incContact1 = incEdge[0].vertex;
				refContact1 = GeometryAlgorithm2D::pointToLineSegment(refEdge[0], refEdge[1], incEdge[0].vertex);
			}
			if (!incEdge[1].isClip)
			{
				incContact2 = incEdge[1].vertex;
				refContact2 = GeometryAlgorithm2D::pointToLineSegment(refEdge[0], refEdge[1], incEdge[1].vertex);
			}

			if (swap)
			{
				std::swap(incContact1, refContact1);
				std::swap(incContact2, refContact2);
			}

			pair.addContact(refContact1, incContact1);
			pair.addContact(refContact2, incContact2);
		}
		return pair;
	}

	ContactPair Narrowphase::clipPolygonPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                            const Feature& featureA, const Feature& featureB, CollisionInfo& info)
	{
		auto polygonA = static_cast<const Polygon*>(shapeA.shape);
		auto polygonB = static_cast<const Polygon*>(shapeB.shape);

		const Vector2 va1 = shapeA.transform.translatePoint(polygonA->vertices()[featureA.index[0]]);
		const Vector2 va2 = shapeA.transform.translatePoint(polygonA->vertices()[featureA.index[1]]);

		const Vector2 vb1 = shapeB.transform.translatePoint(polygonB->vertices()[featureB.index[0]]);
		const Vector2 vb2 = shapeB.transform.translatePoint(polygonB->vertices()[featureB.index[1]]);

		return clipTwoEdge(va1, va2, vb1, vb2, info);
	}

	ContactPair Narrowphase::clipPolygonEdge(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                         const Feature& featureA, const Feature& featureB, CollisionInfo& info)
	{
		auto polygonA = static_cast<const Polygon*>(shapeA.shape);
		auto edgeB = static_cast<const Edge*>(shapeB.shape);

		const Vector2 va1 = shapeA.transform.translatePoint(polygonA->vertices()[featureA.index[0]]);
		const Vector2 va2 = shapeA.transform.translatePoint(polygonA->vertices()[featureA.index[1]]);
		const Vector2 vb1 = shapeB.transform.translatePoint(edgeB->startPoint());
		const Vector2 vb2 = shapeB.transform.translatePoint(edgeB->endPoint());

		return clipTwoEdge(va1, va2, vb1, vb2, info);
	}

	ContactPair Narrowphase::clipPolygonCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                            const Feature& featureA, const Feature& featureB, CollisionInfo& info)
	{
		ContactPair pair;

		auto polygonA = static_cast<const Polygon*>(shapeA.shape);

		const Vector2 va1 = shapeA.transform.translatePoint(polygonA->vertices()[featureA.index[0]]);
		const Vector2 va2 = shapeA.transform.translatePoint(polygonA->vertices()[featureA.index[1]]);

		const Vector2 localB1 = shapeB.transform.inverseTranslatePoint(featureB.vertex[0]);

		auto capsule = static_cast<const Capsule*>(shapeB.shape);
		const real halfWidth = capsule->halfWidth();
		const real halfHeight = capsule->halfHeight();

		real lhs = Math::abs(localB1.y);
		real rhs = halfHeight - halfWidth;

		if (halfWidth > halfHeight)
		{
			lhs = Math::abs(localB1.x);
			rhs = halfWidth - halfHeight;
		}

		const bool isVertexB = fuzzyRealEqual(lhs, rhs, Constant::TrignometryEpsilon);

		const bool isEdgeB = featureB.index[0] == 1 || featureB.index[1] == 1;

		if (isEdgeB || isVertexB)
		{
			Vector2 localB = localB1;

			if (halfWidth > halfHeight)
				localB.x = -localB.x;
			else
				localB.y = -localB.y;

			Vector2 vb2 = shapeB.transform.translatePoint(localB);

			pair = clipTwoEdge(va1, va2, featureB.vertex[0], vb2, info);
		}
		else
		{
			pair = clipEdgeVertex(va1, va2, featureB.vertex[0], info);
			//check if another vertex is valid.
			Vector2 vb1 = shapeB.transform.inverseTranslatePoint(featureB.vertex[0]);
			Vector2 vb2(halfWidth, halfHeight - halfWidth);
			if (halfWidth > halfHeight)
				vb2.set(halfWidth - halfHeight, halfHeight);

			vb2.matchSign(vb1);

			if (halfWidth > halfHeight)
				vb2.x = -vb2.x;
			else
				vb2.y = -vb2.y;

			vb2 = shapeB.transform.translatePoint(vb2);

			Vector2 b = vb2 - va1;

			if (!Math::sameSign(b.dot(va2 - va1), b.dot(vb2 - va2)) &&
				GeometryAlgorithm2D::isPointOnSameSide(va1, va2, va1 + info.normal, vb2))
				pair.addContact(GeometryAlgorithm2D::pointToLineSegment(va1, va2, vb2), vb2);
		}

		return pair;
	}

	ContactPair Narrowphase::clipPolygonRound(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                          const Feature& featureA, const Feature& featureB, CollisionInfo& info)
	{
		auto polygonA = static_cast<const Polygon*>(shapeA.shape);
		const Vector2 va1 = shapeA.transform.translatePoint(polygonA->vertices()[featureA.index[0]]);
		const Vector2 va2 = shapeA.transform.translatePoint(polygonA->vertices()[featureA.index[1]]);

		ContactPair pair = clipEdgeVertex(va1, va2, featureB.vertex[0], info);

		return pair;
	}

	ContactPair Narrowphase::clipEdgeCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                         const Feature& featureA, const Feature& featureB, CollisionInfo& info)
	{
		ContactPair pair;
		const Vector2 localB1 = shapeB.transform.inverseTranslatePoint(featureB.vertex[0]);

		auto capsule = static_cast<const Capsule*>(shapeB.shape);

		auto edgeA = static_cast<const Edge*>(shapeA.shape);
		const Vector2 va1 = shapeA.transform.translatePoint(edgeA->startPoint());
		const Vector2 va2 = shapeA.transform.translatePoint(edgeA->endPoint());

		const real halfWidth = capsule->halfWidth();
		const real halfHeight = capsule->halfHeight();
		real lhs = Math::abs(localB1.y);
		real rhs = halfHeight - halfWidth;
		if (halfWidth > halfHeight)
		{
			lhs = Math::abs(localB1.x);
			rhs = halfWidth - halfHeight;
		}

		const bool isVertexB = fuzzyRealEqual(lhs, rhs, Constant::TrignometryEpsilon);

		const bool isEdgeB = featureB.index[0] == 1 || featureB.index[1] == 1;

		if (isEdgeB || isVertexB)
		{
			//numerical error
			Vector2 localB = localB1;

			if (halfWidth > halfHeight)
				localB.x = -localB.x;
			else
				localB.y = -localB.y;

			Vector2 vb2 = shapeB.transform.translatePoint(localB);


			pair = clipTwoEdge(va1, va2, featureB.vertex[0], vb2, info);
		}
		else
		{
			pair = clipEdgeVertex(va1, va2, featureB.vertex[0], info);
			//check if another vertex is valid.
			Vector2 vb1 = shapeB.transform.inverseTranslatePoint(featureB.vertex[0]);
			Vector2 vb2(halfWidth, halfHeight - halfWidth);
			if (halfWidth > halfHeight)
				vb2.set(halfWidth - halfHeight, halfHeight);

			vb2.matchSign(vb1);

			if (halfWidth > halfHeight)
				vb2.x = -vb2.x;
			else
				vb2.y = -vb2.y;

			vb2 = shapeB.transform.translatePoint(vb2);

			Vector2 b = vb2 - va1;

			if (!Math::sameSign(b.dot(va2 - va1), b.dot(vb2 - va2)) &&
				GeometryAlgorithm2D::isPointOnSameSide(va1, va2, va1 + info.normal, vb2))
				pair.addContact(GeometryAlgorithm2D::pointToLineSegment(va1, va2, vb2), vb2);
		}

		return pair;
	}

	ContactPair Narrowphase::clipEdgeRound(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                       const Feature& featureA, const Feature& featureB, CollisionInfo& info)
	{
		auto edgeA = static_cast<const Edge*>(shapeA.shape);
		const Vector2 va1 = shapeA.transform.translatePoint(edgeA->startPoint());
		const Vector2 va2 = shapeA.transform.translatePoint(edgeA->endPoint());

		ContactPair pair = clipEdgeVertex(va1, va2, featureB.vertex[1], info);

		return pair;
	}

	ContactPair Narrowphase::clipCapsuleCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                            const Feature& featureA, const Feature& featureB, CollisionInfo& info)
	{
		ContactPair pair;

		const Vector2 localA1 = shapeA.transform.inverseTranslatePoint(featureA.vertex[0]);
		const Vector2 localB1 = shapeB.transform.inverseTranslatePoint(featureB.vertex[0]);

		const bool isEdgeA = featureA.index[0] == 1 || featureA.index[1] == 1;
		const bool isEdgeB = featureB.index[0] == 1 || featureB.index[1] == 1;


		auto capsuleA = static_cast<const Capsule*>(shapeA.shape);
		auto capsuleB = static_cast<const Capsule*>(shapeB.shape);

		//test for a
		real halfWidth = capsuleA->halfWidth();
		real halfHeight = capsuleA->halfHeight();

		real lhs = Math::abs(localA1.y);
		real rhs = halfHeight - halfWidth;

		if (halfWidth > halfHeight)
		{
			lhs = Math::abs(localA1.x);
			rhs = halfWidth - halfHeight;
		}

		const bool isVertexA = fuzzyRealEqual(lhs, rhs, Constant::TrignometryEpsilon);

		//test for b
		halfWidth = capsuleB->halfWidth();
		halfHeight = capsuleB->halfHeight();

		lhs = Math::abs(localB1.y);
		rhs = halfHeight - halfWidth;

		if (halfWidth > halfHeight)
		{
			lhs = Math::abs(localB1.x);
			rhs = halfWidth - halfHeight;
		}


		const bool isVertexB = fuzzyRealEqual(lhs, rhs, Constant::TrignometryEpsilon);

		enum class Oper
		{
			ROUND_ROUND = 1,
			ROUND_EDGE = 2,
			EDGE_ROUND = 3,
			EDGE_EDGE = 4
		};
		auto oper = Oper::ROUND_ROUND;


		if (!isEdgeA && !isEdgeB)
		{
			//numerical problem
			if (isVertexA && isVertexB)
				oper = Oper::EDGE_EDGE;
			else if (!isVertexA && isVertexB)
				oper = Oper::ROUND_EDGE;
			else if (isVertexA && !isVertexB)
				oper = Oper::EDGE_ROUND;
		}
		else if (isEdgeA && isEdgeB)
		{
			oper = Oper::EDGE_EDGE;
		}
		else if (isEdgeA && !isEdgeB)
		{
			//edge to vertex case
			oper = Oper::EDGE_ROUND;
			//numerical problem
			if (isVertexB)
				oper = Oper::EDGE_EDGE;
		}
		else if (!isEdgeA && isEdgeB)
		{
			//vertex to edge case
			oper = Oper::ROUND_EDGE;
			if (isVertexA)
				oper = Oper::EDGE_EDGE;
		}
		switch (oper)
		{
		case Oper::ROUND_ROUND:
			pair = clipRoundRound(shapeA, shapeB, featureA, featureB, info);
			break;
		case Oper::ROUND_EDGE:
			info.normal.negate();
			pair = clipEdgeVertex(featureB.vertex[0], featureB.vertex[1], featureA.vertex[0], info);
			info.normal.negate();
			std::swap(pair.points[0], pair.points[1]);
			break;
		case Oper::EDGE_ROUND:
			pair = clipEdgeVertex(featureA.vertex[0], featureA.vertex[1], featureB.vertex[0], info);
			break;
		case Oper::EDGE_EDGE:
			const Vector2 va1 = featureA.vertex[0];
			const Vector2 vb1 = featureB.vertex[0];

			Vector2 localA = localA1;

			if (capsuleA->halfWidth() > capsuleA->halfHeight())
				localA.x = -localA.x;
			else
				localA.y = -localA.y;

			Vector2 localB = localB1;

			if (capsuleB->halfWidth() > capsuleB->halfHeight())
				localB.x = -localB.x;
			else
				localB.y = -localB.y;

			const Vector2 va2 = shapeA.transform.translatePoint(localA);
			const Vector2 vb2 = shapeB.transform.translatePoint(localB);

			pair = clipTwoEdge(va1, va2, vb1, vb2, info);
			break;
		}
		return pair;
	}

	ContactPair Narrowphase::clipCapsuleRound(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                          const Feature& featureA, const Feature& featureB, CollisionInfo& info)
	{
		ContactPair pair;
		const Vector2 localA1 = shapeA.transform.inverseTranslatePoint(featureA.vertex[0]);

		const Vector2 localA2 = shapeA.transform.inverseTranslatePoint(featureA.vertex[1]);

		const bool isSeperateA = !localA1.isSameQuadrant(localA2);

		const bool isEdgeA = featureA.index[0] == 1 || featureA.index[1] == 1;
		if (isEdgeA || isSeperateA)
		{
			//numerical error
			pair = clipEdgeVertex(featureA.vertex[0], featureA.vertex[1], featureB.vertex[0], info);
		}
		else
		{
			pair = clipRoundRound(shapeA, shapeB, featureA, featureB, info);
		}

		return pair;
	}

	ContactPair Narrowphase::clipRoundRound(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                        const Feature& featureA, const Feature& featureB, CollisionInfo& info)
	{
		//need to fix old info
		ContactPair pair;

		const Vector2 v1 = (info.simplex.vertices[0].point[1] - info.simplex.vertices[0].point[0]).normal();
		const Vector2 v2 = (info.simplex.vertices[1].point[1] - info.simplex.vertices[1].point[0]).normal();

		if (v1.dot(info.normal) > v2.dot(info.normal))
			pair.addContact(info.simplex.vertices[0].point[0], info.simplex.vertices[0].point[1]);
		else
			pair.addContact(info.simplex.vertices[1].point[0], info.simplex.vertices[1].point[1]);

		const Vector2 newNormal = pair.points[1] - pair.points[0];
		info.penetration = newNormal.length();
		const real res = newNormal.dot(info.normal);
		info.normal = newNormal.normal();
		if (res < 0)
		{
			std::swap(pair.points[0], pair.points[1]);
			info.normal.negate();
		}

		return pair;
	}

	ContactPair Narrowphase::clipEdgeVertex(const Vector2& va1, const Vector2& va2, const Vector2& vb,
	                                        CollisionInfo& info)
	{
		ContactPair pair;

		Vector2 pA = vb - info.normal * info.penetration;
		const Vector2 edge = (va2 - va1).normal();

		const real checkZero = edge.dot(info.normal);
		if (Math::abs(checkZero) < Constant::TrignometryEpsilon)
			pair.addContact(pA, vb);
		else
		{
			const Vector2 realPa = (pA - va1).lengthSquare() > (pA - va2).lengthSquare() ? va2 : va1;
			pair.addContact(realPa, realPa + info.normal * info.penetration);
		}

		return pair;
	}

	void Narrowphase::polytopeIterNext(std::list<SimplexVertexWithOriginDistance>::iterator& targetIter,
	                                   std::list<SimplexVertexWithOriginDistance>& list)
	{
		++targetIter;
		if (targetIter == list.end())
			targetIter = list.begin();
	}

	void Narrowphase::polytopeIterPrev(std::list<SimplexVertexWithOriginDistance>::iterator& targetIter,
	                                   std::list<SimplexVertexWithOriginDistance>& list)
	{
		if (targetIter == list.begin())
			targetIter = list.end();
		--targetIter;
	}

	void Narrowphase::buildPolytopeFromSimplex(std::list<SimplexVertexWithOriginDistance>& polytope,
	                                           const Simplex& simplex)
	{
		for (auto iter = simplex.vertices.begin(); iter != simplex.vertices.end(); ++iter)
		{
			auto next = iter + 1;
			if (next == simplex.vertices.end())
				next = simplex.vertices.begin();
			SimplexVertexWithOriginDistance pair;
			pair.vertex = *iter;
			pair.distance = GeometryAlgorithm2D::pointToLineSegment(iter->result, next->result, {0, 0})
				.lengthSquare(); //use lengthSquare() to avoid sqrt
			polytope.emplace_back(pair);
		}
	}
}
