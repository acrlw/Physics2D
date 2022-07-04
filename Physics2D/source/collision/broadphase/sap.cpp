#include "../../../include/collision/broadphase/sap.h"

#include "../../../include/dynamics/body.h"
#include "../../../include/collision/algorithm/sat.h"
namespace Physics2D
{
	std::vector<std::pair<Body*, Body*>> SweepAndPrune::generate(const std::vector<Body*>& bodyList)
	{
		std::vector<std::pair<Body*, Body*>> result;
		//sort by x axis
		std::vector<std::pair<Body*, AABB>> bodyBoxPairList;
		bodyBoxPairList.reserve(bodyList.size());

		for(auto&& elem: bodyList)
			bodyBoxPairList.emplace_back(std::make_pair(elem, elem->aabb()));
		
		std::vector<std::pair<Body*, AABB>> sortXAxis = bodyBoxPairList;
		std::vector<std::pair<Body*, AABB>> sortYAxis = bodyBoxPairList;
		std::sort(sortXAxis.begin(), sortXAxis.end(), [](const std::pair<Body*, AABB>& left, const std::pair<Body*, AABB>& right)
			{
				return left.second.minimumX() < right.second.minimumX();
			});
		std::sort(sortYAxis.begin(), sortYAxis.end(), [](const std::pair<Body*, AABB>& left, const std::pair<Body*, AABB>& right)
			{
				return left.second.minimumY() < right.second.minimumY();
			});

		
		std::vector<Body::Relation> xPairs;
		std::vector<Body::Relation> yPairs;

		for (auto before = sortXAxis.begin(); before != sortXAxis.end(); ++before)
		{
			for (auto next = std::next(before); next != sortXAxis.end(); ++next)
			{
				const real minBefore = before->second.minimumX();
				const real maxBefore = before->second.maximumX();
				const real minNext = next->second.minimumX();
				const real maxNext = next->second.maximumX();

				if (!(maxBefore < minNext || maxNext < minBefore))
					xPairs.emplace_back(Body::Relation::generateRelation(before->first, next->first));
				else
					break;
			}
		}

		for (auto before = sortYAxis.begin(); before != sortYAxis.end(); ++before)
		{
			for (auto next = std::next(before); next != sortYAxis.end(); ++next)
			{
				const real minBefore = before->second.minimumY();
				const real maxBefore = before->second.maximumY();
				const real minNext = next->second.minimumY();
				const real maxNext = next->second.maximumY();

				if (!(maxBefore < minNext || maxNext < minBefore))
					yPairs.emplace_back(Body::Relation::generateRelation(before->first, next->first));
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
			if (xPair->relationID == yPair->relationID && (xPair->bodyA->bitmask() & xPair->bodyB->bitmask()))
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

		std::vector<std::pair<Body*, AABB>> bodyBoxPairList;
		bodyBoxPairList.reserve(bodyList.size());

		for (auto&& elem : bodyList)
			bodyBoxPairList.emplace_back(std::make_pair(elem, elem->aabb()));

		//brute search
		for (auto&& elem : bodyBoxPairList)
			if (region.collide(elem.second))
				result.emplace_back(elem.first);

		//std::vector<std::pair<Body*, AABB>> sortXAxis = bodyBoxPairList;
		//std::vector<std::pair<Body*, AABB>> sortYAxis = bodyBoxPairList;
		//std::sort(sortXAxis.begin(), sortXAxis.end(), [](const std::pair<Body*, AABB>& left, const std::pair<Body*, AABB>& right)
		//	{
		//		return left.second.bottomLeft().x < right.second.bottomLeft().x;
		//	});
		//std::sort(sortYAxis.begin(), sortYAxis.end(), [](const std::pair<Body*, AABB>& left, const std::pair<Body*, AABB>& right)
		//	{
		//		return left.second.bottomLeft().y < right.second.bottomLeft().y;
		//	});
		////query x axis
		//size_t beginIndex = 0;
		//size_t endIndex = sortXAxis.size() - 1;
		//size_t indexLower = endIndex;
		//real regionXMin = region.bottomLeft().x;
		//real regionXMax = region.bottomRight().x;
		//while(beginIndex < indexLower)
		//{
		//	auto iter = sortXAxis.begin();
		//	indexLower = (beginIndex + endIndex) / 2;
		//	std::advance(iter, indexLower);
		//	real xMin = iter->second.bottomLeft().x;
		//	if(xMin > regionXMin)
		//	{
		//		//narrow right side
		//		endIndex = indexLower;
		//	}
		//	else if (xMin < regionXMin)
		//	{
		//		//narrow left side
		//		beginIndex = indexLower;
		//	}

		//}
		//beginIndex = 0;
		//endIndex = sortXAxis.size() - 1;
		//size_t indexUpper = beginIndex;
		//while (endIndex > indexUpper)
		//{
		//	auto iter = sortXAxis.begin();
		//	indexUpper = (beginIndex + endIndex) / 2;
		//	std::advance(iter, indexUpper);
		//	real xMax = iter->second.bottomRight().x;
		//	if (xMax > regionXMax)
		//	{
		//		//narrow right side
		//		endIndex = indexUpper;
		//	}
		//	else if (xMax < regionXMax)
		//	{
		//		beginIndex = indexUpper;
		//	}

		//}
		//for(size_t i = indexLower; i <= indexUpper; ++i)
		//	result.emplace_back(sortXAxis[i].first);
		////sort by body id
		//auto yAxisBegin = sortYAxis.begin();
		//real regionYMin = region.bottomLeft().y;
		//real regionYMax = region.topLeft().y;

		////query y axis

		////sort by body id

		////compare two result, using double pointer



		return result;
	}
}
