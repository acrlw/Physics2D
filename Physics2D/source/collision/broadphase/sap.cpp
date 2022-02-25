#include "../../../include/collision/broadphase/sap.h"

#include "../../../include/dynamics/body.h"

namespace Physics2D
{
	std::vector<std::pair<Body*, Body*>> SweepAndPrune::generate(const std::vector<Body*>& bodyList)
	{
		std::vector<std::pair<Body*, Body*>> result;
		//sort by x axis
		std::vector<Body*> sortXAxis = bodyList;
		std::vector<Body*> sortYAxis = bodyList;
		std::sort(sortXAxis.begin(), sortXAxis.end(), [](Body* left, Body* right)
			{
				return left->position().x < right->position().x;
			});
		std::sort(sortYAxis.begin(), sortYAxis.end(), [](Body* left, Body* right)
			{
				return left->position().y < right->position().y;
			});

		
		

		std::vector<Body::Relation> xPairs;
		std::vector<Body::Relation> yPairs;

		for (auto before = sortXAxis.begin(); before != sortXAxis.end(); ++before)
		{
			AABB aabbBefore = (*before)->aabb();
			for (auto next = std::next(before); next != sortXAxis.end(); ++next)
			{
				AABB aabbNext = (*next)->aabb();

				const real minBefore = aabbBefore.bottomLeft().x;
				const real maxBefore = aabbBefore.bottomRight().x;
				const real minNext = aabbNext.bottomLeft().x;
				const real maxNext = aabbNext.bottomRight().x;

				if (!(maxBefore < minNext || maxNext < minBefore))
					xPairs.emplace_back(Body::Relation::generateRelation(*before, *next));
				else
					break;
			}
		}

		for (auto before = sortYAxis.begin(); before != sortYAxis.end(); ++before)
		{
			AABB aabbBefore = (*before)->aabb();
			for (auto next = std::next(before); next != sortYAxis.end(); ++next)
			{
				AABB aabbNext = (*next)->aabb();

				const real minBefore = aabbBefore.bottomLeft().y;
				const real maxBefore = aabbBefore.topLeft().y;
				const real minNext = aabbNext.bottomLeft().y;
				const real maxNext = aabbNext.topLeft().y;

				if (!(maxBefore < minNext || maxNext < minBefore))
					yPairs.emplace_back(Body::Relation::generateRelation(*before, *next));
				else
					break;
			}
		}

		std::sort(xPairs.begin(), xPairs.end(), [](const Body::Relation& left, const Body::Relation& right)
			{
				return left.relationID < right.relationID;
			});
		std::sort(yPairs.begin(), yPairs.end(), [](const Body::Relation& left, const Body::Relation& right)
			{
				return left.relationID < right.relationID;
			});

		//double pointer check
		auto xPair = xPairs.begin();
		auto yPair = yPairs.begin();
		while (xPair != xPairs.end() && yPair != yPairs.end())
		{
			if (xPair->relationID == yPair->relationID)
			{
				result.emplace_back(std::make_pair(xPair->bodyA, xPair->bodyB));
				xPair = std::next(xPair);
				yPair = std::next(yPair);
			}
			else if(xPair->relationID > yPair->relationID)
				yPair = std::next(yPair);
			else
				xPair = std::next(xPair);
		}


		return result;
	}

	std::vector<Body*> SweepAndPrune::query(const std::vector<Body*>& bodyList, const AABB& region)
	{
		std::vector<Body*> result;

		std::vector<Body*> sortXAxis = bodyList;
		std::sort(sortXAxis.begin(), sortXAxis.end(), [](Body* left, Body* right)
			{
				return left->position().x < right->position().x;
			});

		auto lowerIter = sortXAxis.begin();
		auto upperIter = sortXAxis.end();
		{
			auto headIter = sortXAxis.begin();
			auto mid = sortXAxis.size();
			while (mid > 0)
			{
				mid /= 2;
				std::advance(lowerIter, mid);
				if ((*lowerIter)->aabb().bottomRight().x > region.bottomLeft().x)
					lowerIter = headIter;
				else
					headIter = lowerIter;

			}

			auto tailIter = sortXAxis.end();
			mid = sortXAxis.size();

			while (mid > 0)
			{
				mid /= 2;
				std::advance(upperIter, -mid);
				if ((*upperIter)->aabb().bottomLeft().x < region.bottomRight().x)
					upperIter = tailIter;
				else
					tailIter = upperIter;
			}
		}
		auto finalLower = lowerIter;
		auto finalUpper = upperIter;
		std::sort(lowerIter, upperIter, [](Body* left, Body* right)
			{
				return left->position().y < right->position().y;
			});
		{
			auto headIter = lowerIter;
			auto size = std::distance(lowerIter, upperIter);
			auto mid = size;
			while (mid > 0)
			{
				mid /= 2;
				std::advance(finalLower, mid);
				if ((*finalLower)->aabb().topLeft().y > region.bottomLeft().y)
					finalLower = headIter;
				else
					headIter = finalLower;

			}

			auto tailIter = upperIter;
			mid = size;

			while (mid > 0)
			{
				mid /= 2;
				std::advance(finalUpper, -mid);
				if ((*finalUpper)->aabb().bottomRight().y < region.topLeft().y)
					finalUpper = tailIter;
				else
					tailIter = finalUpper;
			}
		}

		for (auto iter = finalLower; iter <= finalUpper; ++iter)
			if((*iter)->aabb().collide(region))
				result.emplace_back(*iter);
		return result;
	}
}
